#include <cstdlib>
#include <iostream>

template <typename T>
struct PointCloud
{
    struct Point
    {
        T  x,y,z;
    };

    std::vector<Point>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

template <typename T>
void generateRandomPointCloud(PointCloud<T> &point, const size_t N, const T max_range = 10)
{
    // Generating Random Point Cloud
    point.pts.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        point.pts[i].x = max_range * (rand() % 1000) / T(1000);
        point.pts[i].y = max_range * (rand() % 1000) / T(1000);
        point.pts[i].z = max_range * (rand() % 1000) / T(1000);
    }
}

// This is an exampleof a custom data set class
template <typename T>
struct PointCloud_Quat
{
    struct Point
    {
        T  w,x,y,z;
    };

    std::vector<Point>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].w;
        else if (dim==1) return pts[idx].x;
        else if (dim==2) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

template <typename T>
void generateRandomPointCloud_Quat(PointCloud_Quat<T> &point, const size_t N)
{
    // Generating Random Quaternions
    point.pts.resize(N);
    T theta, X, Y, Z, sinAng, cosAng, mag;
    for (size_t i=0;i<N;i++)
    {
        theta = M_PI * (((double)rand()) / RAND_MAX);
        // Generate random value in [-1, 1]
        X = 2 * (((double)rand()) / RAND_MAX) - 1;
        Y = 2 * (((double)rand()) / RAND_MAX) - 1;
        Z = 2 * (((double)rand()) / RAND_MAX) - 1;
        mag = sqrt(X*X + Y*Y + Z*Z);
        X /= mag; Y /= mag; Z /= mag;
        cosAng = cos(theta / 2);
        sinAng = sin(theta / 2);
        point.pts[i].w = cosAng;
        point.pts[i].x = X * sinAng;
        point.pts[i].y = Y * sinAng;
        point.pts[i].z = Z * sinAng;
    }
}

// This is an exampleof a custom data set class
template <typename T>
struct PointCloud_Orient
{
    struct Point
    {
        T  theta;
    };

    std::vector<Point>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim = 0) const
    {
        return pts[idx].theta;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

template <typename T>
void generateRandomPointCloud_Orient(PointCloud_Orient<T> &point, const size_t N)
{
    // Generating Random Orientations
    point.pts.resize(N);
    for (size_t i=0;i<N;i++) {
        point.pts[i].theta = ( 2 * M_PI * (((double)rand()) / RAND_MAX) ) - M_PI;
    }
}


