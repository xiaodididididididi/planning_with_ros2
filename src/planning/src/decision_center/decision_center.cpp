#include "decision_center.h"

namespace Planning
{
    DecisionCenter::DecisionCenter()
    {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "decision_center created");

        decision_config_ = std::make_unique<ConfigReader>();
        decision_config_->read_decision_config();
    }

    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car,
                                            const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        if (obses.empty())
        {
            return;
        }

        sl_points_.clear();
        const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;
        const double right_bound_l = -decision_config_->pnc_map().road_half_width_ / 2.0;
        const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ - 50);
        const double least_length = std::max(car->ds_dt() * dis_time, 30.0);
        const double referline_end_length = decision_config_->refer_line().front_size_ * decision_config_->pnc_map().segment_len_;
        SLPoint p;

        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->s() - car->s();
            if (obs_dis_s > referline_end_length || obs_dis_s < -least_length)
            {
                continue;
            }

            if (obs->l() > right_bound_l && obs->l() < left_bound_l && fabs(obs->dl_dt()) < min_speed && obs->ds_dt() < car->ds_dt() / 2.0)
            {
                p.s_ = obs->s() + obs->ds_dt() * obs_dis_s / (car->ds_dt() - obs->ds_dt());
                const double obs_left_bound_l = obs->l() + obs->width() / 2.0;
                const double obs_right_bound_l = obs->l() - obs->width() / 2.0;
                const double left_width = left_bound_l - obs_left_bound_l;
                const double right_width = obs_right_bound_l - right_bound_l;

                if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0)
                {
                    p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
                    p.type_ = static_cast<int>(SLPointType::LFET_PASS);
                    sl_points_.emplace_back(p);
                }
                else
                {
                    if (right_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0)
                    {
                        p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
                        p.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
                        sl_points_.emplace_back(p);
                    }
                    else
                    {
                        p.l_ = 0.0;
                        p.s_ = obs->s() - decision_config_->decision().safe_dis_s_;
                        p.type_ = static_cast<int>(SLPointType::STOP);
                        sl_points_.emplace_back(p);
                        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "-------------stop obs,p:(s=%.2f,l=%.2f)", p.s_, p.l_);
                        break;
                    }
                }
            }
        }

        if (sl_points_.empty())
        {
            return;
        }

        SLPoint p_start;
        p_start.s_ = sl_points_[0].s_ - least_length;
        p_start.l_ = 0.0;
        p_start.type_ = static_cast<int>(SLPointType::START);
        sl_points_.emplace(sl_points_.begin(), p_start);

        if (sl_points_.back().type_ != static_cast<int>(SLPointType::STOP))
        {
            SLPoint p_end;
            p_end.s_ = sl_points_.back().s_ + least_length;
            p_end.l_ = 0.0;
            p_end.type_ = static_cast<int>(SLPointType::END);
            sl_points_.emplace_back(p_end);
        }
    }

    void DecisionCenter::make_speed_decision(const std::shared_ptr<VehicleBase> &car, const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        if (obses.empty())
        {
            return;
        }

        st_points_.clear();
        const double ori_dis_time = static_cast<double>(decision_config_->local_speeds().speed_size_ - 50);
        const double ori_dis = ori_dis_time * decision_config_->main_car().speed_ori_;
        const double real_break_time = (ori_dis_time + static_cast<double>(decision_config_->local_speeds().speed_size_)) / 2.0;
        STPoint p;

        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->s_2path() + car->speed();
            if (obs_dis_s > ori_dis || obs_dis_s < -decision_config_->decision().safe_dis_s_)
            {
                continue;
            }

            double t_in;
            double t_out;

            if (fabs(obs->l_2path()) < obs->width() / 2.0)
            {
                if (fabs(obs->dl_dt_2path()) < min_speed)
                {
                    if (obs->ds_dt_2path() > car->speed() + 0.5)
                    {
                        continue;
                    }

                    obs->update_t0();
                    p.t0_ = obs->t0();
                    p.s0_ = obs_dis_s + obs->ds_dt_2path() * p.t0_ - ori_dis;
                    t_in = 0.0;
                    t_out = decision_config_->local_speeds().speed_size_;
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------obs_dis_s=%.2f,p.t0=%.2f,p.so=%.2f,t_in=%.2f,t_out=%.2f",
                                obs_dis_s, p.t0_, p.s0_, t_in, t_out);

                    p.t_ = p.t0_ + real_break_time;
                    p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_ + obs->ds_dt_2path() * p.t_;
                    p.ds_dt_2path_ = obs->ds_dt_2path();
                    p.type_ = static_cast<int>(STPointType::STOP);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------stop or follow obs,p:(s=%.2f,t=%.2f,ds_dt=%.2f)", p.s_2path_, p.t_, p.ds_dt_2path_);

                    obs->update_t_in_out(p.t_, t_in, t_out);
                    break;
                }
            }

            else
            {
                if (fabs(obs->dl_dt_2path()) < min_speed)
                {
                    continue;
                }

                if (decision_config_->main_car().speed_ori_ < min_speed)
                {
                    continue;
                }

                const double car_dis_time = obs_dis_s / decision_config_->main_car().speed_ori_;
                const double obs_dis_time = (0.0 - obs->l_2path()) / obs->dl_dt_2path();
                if (obs_dis_time < 0.0)
                {
                    continue;
                }

                obs->update_t0();
                p.t0_ = obs->t0();
                p.s0_ = obs_dis_s - ori_dis;

                const double delta_t = decision_config_->decision().safe_dis_s_ / decision_config_->main_car().speed_ori_;
                const double half_through_time = (-obs->length() / 2.0) / obs->dl_dt_2path();
                t_in = obs_dis_time - half_through_time;
                t_out = obs_dis_time + half_through_time;
                RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------obs_dis_s= %.2f, p.t0= %.2f, p.s0= %.2f, car_dis_time= %.2f, obs_dis_time= %.2f, t_in= %.2f, t_out= %.2f,delta_t= %.2f",
                            obs_dis_s, p.t0_, p.s0_, car_dis_time, obs_dis_time, t_in, t_out, delta_t);

                if (car_dis_time > obs_dis_time && car_dis_time < t_out + delta_t)
                {
                    p.t_ = t_out;
                    p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::GIVE_WAY);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------give way, p:(s= %.2f, t=%.2f, ds_dt= %.2f)",
                                p.s_2path_, p.t_, p.ds_dt_2path_);
                }

                else if (car_dis_time < obs_dis_time && car_dis_time > t_in - delta_t)
                {
                    p.t_ = t_in;
                    p.s_2path_ = obs_dis_s + decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::RUSH_OUT);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------rush out, p:(s= %.2f, t=%.2f, ds_dt= %.2f)",
                                p.s_2path_, p.t_, p.ds_dt_2path_);
                }

                obs->update_t_in_out(p.t_, t_in, t_out);
            }
        }

        if (st_points_.empty())
        {
            return;
        }

        STPoint p_start;
        p_start.t_ = st_points_[0].t0_;
        p_start.s_2path_ = st_points_[0].s0_;
        p_start.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
        p_start.type_ = static_cast<int>(STPointType::START);
        st_points_.emplace(st_points_.begin(), p_start);

        STPoint p_end;
        p_end.t_ = decision_config_->local_speeds().speed_size_;
        p_end.s_2path_ = st_points_.back().s_2path_ + st_points_.back().ds_dt_2path_ * (p_end.t_ - st_points_.back().t_);
        p_end.ds_dt_2path_ = st_points_.back().ds_dt_2path_;
        p_end.type_ = static_cast<int>(STPointType::END);
        st_points_.emplace_back(p_end);
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------p_start:(s= %.2f, t=%.2f, ds_dt= %.2f), p_end:(s= %.2f, t=%.2f, ds_dt= %.2f)",
                    p_start.s_2path_, p_start.t_, p_start.ds_dt_2path_, p_end.s_2path_, p_end.t_, p_end.ds_dt_2path_);
    }
}