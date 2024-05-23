# ROS Rate and Spin function

由于一次实验中出的关于ros 频率设置不对应的问题，写一个总结，记录关于下述的关系，在使用中的注意事项：

* Rate
* SpinOnce
* Spin
* queue_size

## queue_size

**In a publisher:**

```cpp
template<class M>
ros::Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false);
```

`queue_size` [required]

This is the size of the outgoing message queue. If you are publishing faster than roscpp can send the messages over the wire, roscpp will start dropping old messages. A value of 0 here means an infinite queue, which can be dangerous. See the [rospy documentation on choosing a good queue_size](http://wiki.ros.org/rospy/Overview/Publishers and Subscribers#Choosing_a_good_queue_size) for more information.

ros 本身的消息传递需要时间，做极端假设，若 publisher 的 rate frequency 设置的过大，单位时间超过了 ros 发送的时间，则有一部分消息将丢失 drop out。

**to be simple，one message at a fixed rete 可以使用较小的 queue_size；multi msgs in a burst 则应确定队列大小足够容纳，防止丢失。**

queue_size 设置为0，忽略和None(in python) is dangerous.

特别地，上述doc给出了 **queue_size 1,2,3** 的注意：当系统没有过载，能够在`1/rate` 之中拿到 msg，则可以设置成1，2，3。设置成 1 对于高实时性，不考虑历史数据的 sensor 是可以的。

**In a subscriber:**

```cpp
template<class M>
ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size, <callback, which may involve multiple arguments>, const ros::TransportHints& transport_hints = ros::TransportHints());
```

`queue_size`

This is the incoming message queue size roscpp will use for your callback. If messages are arriving too fast and you are unable to keep up, roscpp will start throwing away messages. A value of 0 here means an infinite queue, which can be dangerous.

这个值与 callback 强相关，如果rate过大，数据过快，而回调函数无法处理该 rate 下的数据，则丢包。

Queueing and Lazy Deserialization: 

When a message first arrives on a topic it gets put into a queue whose size is specified by the `queue_size` parameter in `subscribe()`. If the queue is full and a new message arrives, the oldest message will be thrown out. Additionally, the message is not actually deserialized until the first callback which needs it is about to be called.
