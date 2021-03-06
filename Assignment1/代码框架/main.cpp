#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Matrix4f get_view_matrix(Vector3f eye_pos)
{
    Matrix4f view = Matrix4f::Identity();

    Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Matrix4f get_model_matrix(float rotation_angle)
{
    Matrix4f model = Matrix4f::Identity();

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Matrix4f rotate;
    float radiant = rotation_angle * MY_PI / 180.0;
    rotate << cos(radiant), -sin(radiant), 0, 1,
        sin(radiant), cos(radiant), 0, 1,
        0, 0, 1, 1,
        0, 0, 0, 1;

    model = rotate * model;

    return model;
}

Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                               float zNear, float zFar)
{
    // Students will implement this function

    Matrix4f projection = Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.
    float fov_radiant = eye_fov * MY_PI / 180.0;
    float t = abs(zNear) * tan(fov_radiant / 2.0);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;

    Matrix4f ortho, ortho_scale, ortho_translate;
    ortho_scale << 2.0 / (r - l), 0, 0, 0,
        0, 2.0 / (t - b), 0, 0,
        0, 0, -2.0 / (zFar - zNear), 0,
        0, 0, 0, 1;
    ortho_translate << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zFar + zNear) / 2,
        0, 0, 0, 1;
    ortho = ortho_scale * ortho_translate;

    Matrix4f persp2ortho;
    persp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, zFar * -zNear,
        0, 0, 1, 0;

    projection = ortho * persp2ortho * projection;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Vector3f eye_pos = {0, 0, 5};

    std::vector<Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
