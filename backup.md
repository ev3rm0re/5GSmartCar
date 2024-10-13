
```cpp
vector<vector<Point>> contours;
findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

vector<double> center_x;
for (const auto& contour : contours) {
    if (contourArea(contour) < 15) continue;
    RotatedRect rect = minAreaRect(contour);
    double angle = rect.angle;
    double width = rect.size.width;
    double height = rect.size.height;
    if (width < height) {
        angle += 90;
    }
    cout << "angle: " << angle << endl;
    if (angle > 150 || angle < 30) continue;
    Point2f box[4];
    rect.points(box);
    for (int i = 0; i < 4; i++) {
        box[i].y = box[i].y + size_y / 2;
    }
    for (int i = 0; i < 4; i++) {
        line(frame, box[i], box[(i + 1) % 4], Scalar(0, 255, 0), 2);
    }
    center_x.push_back(rect.center.x);
}

cout << "center_x: " << center_x.size() << endl;
double center = size_x / 2;
if (!center_x.empty()) {
    sort(center_x.begin(), center_x.end());
    if (center_x.size() >= 2) {
        center = (center_x[0] + center_x.back()) / 2;
    } else {
        if (center_x[0] < center) {
            center = center_x[0] / 2 + center;
        } else {
            center = center_x[0] / 2;
        }
    }
}
```