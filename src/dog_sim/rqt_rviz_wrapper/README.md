# RQT RViz Wrapper

## Dependencies

- [wmctrl](http://manpages.ubuntu.com/manpages/dapper/man1/wmctrl.1.html) (interact with a EWMH/NetWM compatible X Window Manager)

        sudo apt-get install wmctrl

## Run

You must set the `-d, --disable-init-threads` flag e.g.:

```
rosrun rqt_rviz_wrapper rqt_rviz_wrapper -d

rqt -d
```
