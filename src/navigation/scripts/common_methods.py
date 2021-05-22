from std_msgs.msg import ColorRGBA, String
import rospy


def color_name_from_rgba(color_rgba):

    if color_rgba == ColorRGBA(255 / 255, 255 / 255, 255 / 255, 1):
        return "white"
    elif color_rgba == ColorRGBA(0, 0, 0, 1):
        return "black"
    elif color_rgba == ColorRGBA(255 / 255, 0, 0, 1):
        return "red"
    elif color_rgba == ColorRGBA(0, 0, 255 / 255, 1):
        return "blue"
    elif color_rgba == ColorRGBA(0, 255 / 255, 0, 1):
        return "green"
    elif ColorRGBA(247 / 255, 202 / 255, 24 / 255, 1):
        return "yellow"
    return "unknown color"
