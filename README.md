# Common package for various labs @ Centrale Nantes

The `ecn_common` ROS 2 package ships with various tools used in robotics labs.

## Butterworth filter

Found in `butterworth.h`, allows 1D or n-D filtering at a desired frequency with 2nd order Butterworth filter.

Example:

```
Butterworth_nD filters(3,       // dim = 3
                       2,       // 2 Hz
                       0.1);    // 0.1 s

std::vector<double> value(3, 0.);
// pass by reference to filter the vector
filters.filter(value);
```

## Color detector

Found in `color_detector.h`:

    - init with a given (R,G,B) color
    - setup the camera parameters
    - uses HSV value internally, saturation and value can be tuned with a GUI if `showSegmentation` is True
    - returns the (x, y, area) of the largest detected object


Example:

```
ColorDetector cd;
cd.detectColor(0, 255, 0); // we want green
cd.setCamera(px, py, u0, v0);
cv::Mat im (of some size), im_out;
if(cd.process(im, im_out)
{
    // object has been detected
    cd.x();
    cd.y();
    cd.area();
}

```

## Synchronous service

Found in `srv_sync.h`:

    - build the `ServiceNodeSync` with the service type
    - init with the service address and the timeout
    - call with `std::optional<ResponseT> call(const RequestT &req)`

Example:

```
// typically a member variable
ServiceNodeSync<example_interfaces::srv::AddTwoInts> client
// typically in the constructor
client.init("my_service_server", 100ms);

// typically in the main loop
example_interfaces::srv::AddTwoInts::Request req;
const auto res = client.call(req);
if(res.has_value())
{
    // do stuff with the response
}
```
