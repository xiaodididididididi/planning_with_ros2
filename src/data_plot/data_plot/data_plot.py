import rclpy
from rclpy.node import Node
from base_msgs.msg import PlotInfo
import numpy as np
import matplotlib.pyplot as plt


class PlotData(Node):
    def __init__(self):
        super().__init__("data_plot_node")
        self.get_logger().info("data_plot_node created")

        self.subscription = self.create_subscription(
            PlotInfo,
            "planning/plot_info",
            self.do_plot,
            10,
        )

    def do_plot(self, plot_info):
        plt.clf()

        if not plot_info.trajectory_info.local_trajectory:
            self.get_logger().warn("received empty trajectory info")
            return

        s = np.asarray(
            [point.path_point.s for point in plot_info.trajectory_info.local_trajectory]
        )

        l = np.asarray(
            [point.path_point.l for point in plot_info.trajectory_info.local_trajectory]
        )

        dl_ds = np.asarray(
            [
                point.path_point.dl_ds
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        theta = np.asarray(
            [
                point.path_point.theta
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        kappa = np.asarray(
            [
                point.path_point.kappa
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        t = np.asarray(
            [point.speed_point.t for point in plot_info.trajectory_info.local_trajectory]
        )

        s_2path = np.asarray(
            [
                point.speed_point.s_2path
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        speed = np.asarray(
            [
                point.speed_point.speed
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        acceleration = np.asarray(
            [
                point.speed_point.acceleration
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        fig1 = plt.subplot(4, 1, 1)
        fig2 = plt.subplot(4, 1, 2)
        fig3 = plt.subplot(4, 1, 3)
        fig4 = plt.subplot(4, 1, 4)

        plt.sca(fig1)
        plt.plot(s, l, color="green", label="l", linestyle="solid")

        for obs in plot_info.obs_info:
            s_left = obs.s - obs.obs_length / 2.0
            s_right = obs.s + obs.obs_length / 2.0
            l_bottom = obs.l - obs.obs_width / 2.0
            l_up = obs.l + obs.obs_width / 2.0

            p_sl = plt.Polygon(
                xy=[
                    [s_left, l_bottom],
                    [s_right, l_bottom],
                    [s_right, l_up],
                    [s_left, l_up],
                ],
                color="blue",
                alpha=0.8,
            )
            fig1.add_patch(p_sl)

        plt.title("sl info")
        plt.xlabel("s")
        plt.legend()

        plt.sca(fig2)
        plt.plot(s, dl_ds, color="red", label="dl_ds", linestyle="solid")
        plt.plot(s, theta, color="orange", label="theta", linestyle="solid")
        plt.plot(s, kappa, color="cyan", label="kappa", linestyle="solid")
        plt.title("sl params")
        plt.xlabel("s")
        plt.legend()

        plt.sca(fig3)
        plt.plot(t, s_2path, color="green", label="s_2path", linestyle="solid")
        for obs in plot_info.obs_info:
            delta_s = obs.ds_dt_2path * (obs.t_out - obs.t_in)
            s_left_bottom = obs.s_2path - obs.obs_length / 2.0
            s_left_up = obs.s_2path + obs.obs_length / 2.0
            s_right_bottom = s_left_bottom + delta_s
            s_right_up = s_left_up + delta_s

            p_st = plt.Polygon(
                xy=[
                    [obs.t_in, s_left_bottom],
                    [obs.t_out, s_right_bottom],
                    [obs.t_out, s_right_up],
                    [obs.t_in, s_left_up],
                ],
                color="blue",
                alpha=0.8,
            )
            fig3.add_patch(p_st)

        plt.title("st info")
        plt.xlabel("t")
        plt.legend()

        plt.sca(fig4)
        plt.plot(t, speed, color="red", label="speed", linestyle="solid")
        plt.plot(t, acceleration, color="cyan", label="acceleration", linestyle="solid")
        plt.title("st params")
        plt.xlabel("t")
        plt.legend()

        plt.pause(0.05)


def main(args=None):
    rclpy.init(args=args)
    plot_node = PlotData()

    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
