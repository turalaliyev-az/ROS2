"""Test ordering for this package."""


def pytest_collection_modifyitems(items):
    """
    Run the linters first.

    flake8 forks worker processes; once a test has started ROS (rclpy.init),
    the DDS threads don't survive a fork and the workers hang. The sort is
    stable, so everything else keeps its order.
    """
    items.sort(key=lambda item: item.get_closest_marker('linter') is None)
