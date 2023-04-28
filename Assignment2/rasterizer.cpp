// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f p0,p1,p2;
    p0 << _v[1].x() - _v[0].x() , _v[1].y() - _v[0].y() , 0;
    p1 << _v[2].x() - _v[1].x() , _v[2].y() - _v[1].y() , 0;
    p2 << _v[0].x() - _v[2].x() , _v[0].y() - _v[2].y() , 0;
    char flag = 0;
    
    for(int i = 0 ; i < 3 ; i++){
        Eigen::Vector3f t,p;
        t << _v[(i + 1)%3].x() - _v[i].x() , _v[(i + 1)%3].y() - _v[i].y() , 0;
        p << x - _v[i].x() , y - _v[i].y(), 0;
        if(i == 0){
            if(t.cross(p).z() >= 0)   flag = 1;
            else flag = -1;
        }else{
            if(t.cross(p).z() >= 0 && flag == 1) continue;
            if(t.cross(p).z() < 0 && flag == -1)  continue;
            return false;
        }
    }
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}
static bool insideFloatTriangle(float x, float y, const Vector3f* _v){
        // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f p0,p1,p2;
    p0 << _v[1].x() - _v[0].x() , _v[1].y() - _v[0].y() , 0;
    p1 << _v[2].x() - _v[1].x() , _v[2].y() - _v[1].y() , 0;
    p2 << _v[0].x() - _v[2].x() , _v[0].y() - _v[2].y() , 0;
    char flag = 0;
    
    for(int i = 0 ; i < 3 ; i++){
        Eigen::Vector3f t,p;
        t << _v[(i + 1)%3].x() - _v[i].x() , _v[(i + 1)%3].y() - _v[i].y() , 0;
        p << x - _v[i].x() , y - _v[i].y(), 0;
        if(i == 0){
            if(t.cross(p).z() >= 0)   flag = 1;
            else flag = -1;
        }else{
            if(t.cross(p).z() >= 0 && flag == 1) continue;
            if(t.cross(p).z() < 0 && flag == -1)  continue;
            return false;
        }
    }
    return true;
}

int rst::rasterizer::MASS(int x,int y,const Eigen::Vector3f* _v,const Eigen::Vector3f* color){
    int sum = 0;
    
    return sum;
}
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    float minx = v[0].x() ,miny = v[0].y() , maxx = v[0].x() , maxy = v[0].y();
    for(int i = 0 ; i < 3; i ++){
        if(v[i].x() < minx) minx = v[i].x();
        if(v[i].x() > maxx) maxx = v[i].x();
        if(v[i].y() < miny) miny = v[i].y();
        if(v[i].y() > maxy) maxy = v[i].y();
    }

    Eigen::Vector3f pv[3];
    pv[0] = t.v[0];
    pv[1] = t.v[1];
    pv[2] = t.v[2];
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int xmin = (int)minx,ymin = (int)miny,xmax = (int)(maxx + 1),ymax = (int)(maxy + 1);
    for(int x = xmin; x < xmax ;x++){
        for(int y = ymin; y < ymax ;y++){
            int sum = 0;
            for(int i = 0 ; i < 2;i++){
                for(int j = 0 ;j < 2;j++){
                    //count the pixel(x,y) depth
                    auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5 * i, y + 0.5 * j, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    //judge whethe the sample is in the Triangle and 
                    //compare the sample depth wiht the Z-Buff(min depth)
                    if(insideFloatTriangle(x + i * 0.5 , y + j * 0.5, pv) && z_interpolated < msaa_depth_buf[4 * get_index(x,y) + i * 2 + j]){
                        msaa_depth_buf[4 * get_index(x,y) + i * 2 + j] = z_interpolated;
                        msaa_frame_buf[4 * get_index(x,y) + i * 2 + j] = t.getColor();
                        sum++;
                    }
                }
            }
            if(sum == 0)    continue;
            int ind = get_index(x,y);

            float z = msaa_depth_buf[4 * ind];
            set_pixel(Eigen::Vector3f(x,y,z),(msaa_frame_buf[4 * ind] + msaa_frame_buf[4 * ind + 1] + msaa_frame_buf[4 * ind + 2] + msaa_frame_buf[4 * ind + 3]) / 4);
        }
    }
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    std::fill(msaa_frame_buf.begin(), msaa_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    std::fill(msaa_depth_buf.begin(), msaa_depth_buf.end(), std::numeric_limits<float>::infinity());

}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    msaa_frame_buf.resize(4 * w * h + 5);
    msaa_depth_buf.resize(4 * w * h + 5);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on