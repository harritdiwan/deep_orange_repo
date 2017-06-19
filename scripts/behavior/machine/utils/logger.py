import pickle
import rospy


class Log(object):
    def __init__(self, initial, final, data):
        self.initial = initial
        self.final = final
        self.data = data

    @property
    def short_msg(self):
        diff = len(set(self.initial) & set(self.final))
        return ".".join(self.initial[diff:]) + " -> " + ".".join(self.final[diff:])

    @property
    def long_msg(self):
        return ".".join(self.initial) + " -> " + ".".join(self.final)

    def freeze(self):
        return pickle.dumps(self)


class Logger(object):
    def __init__(self, console_log=False, ros_log=False, filename=None):
        # print to console?
        self.console_log = console_log

        # use ros logging?
        self.ros_log = ros_log

        # write to file?
        if filename is not None:
            # open file
            self.logfile = None
        else:
            self.logfile = None

    def log(self, initial, final, vehicle):
        # a transition happened
        if initial != final:
            log_object = Log(initial, final, vehicle)
            if self.console_log:
                print (log_object.short_msg)
            if self.ros_log:
                rospy.loginfo(log_object.long_msg)
                pass
            if self.logfile is not None:
                # write(log_object.freeze())
                pass
