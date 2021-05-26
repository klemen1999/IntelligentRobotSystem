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


def ring_name_from_vaccine_name(vaccine_name):
    vaccine_dict = {
        'BlacknikV': 'black',
        'Rederna': 'red',
        'Greenzer': 'green',
        'StellaBluera': 'blue'
    }
    return vaccine_dict[vaccine_name]
