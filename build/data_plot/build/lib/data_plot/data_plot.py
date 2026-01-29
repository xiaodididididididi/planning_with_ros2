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

        # t = np.asarray(
        #     [point.path_point.t for point in plot_info.trajectory_info.local_trajectory]
        # )

        # s_2path = np.asarray(
        #     [
        #         point.path_point.s_2path
        #         for point in plot_info.trajectory_info.local_trajectory
        #     ]
        # )

        # speed = np.asarray(
        #     [
        #         point.path_point.speed
        #         for point in plot_info.trajectory_info.local_trajectory
        #     ]
        # )

        # acceleration = np.asarray(
        #     [
        #         point.path_point.acceleration
        #         for point in plot_info.trajectory_info.local_trajectory
        #     ]
        # )

        fig1 = plt.subplot(4, 1, 1)
        fig2 = plt.subplot(4, 1, 2)
        fig3 = plt.subplot(4, 1, 3)
        fig4 = plt.subplot(4, 1, 4)

        plt.sca(fig1)
        plt.plot(s, l, color="green", label="l", linestyle="solid")
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
