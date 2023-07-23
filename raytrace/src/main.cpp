////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>


// Utilities
#include "utils.h"


// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere
using namespace Eigen;


////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////


// Output filename 
const std::string filename("raytrace.png");


// Camera settings
const double focal_length = 10;
const double field_of_view = 0.7854; // 45 degrees
const double image_z = 5;
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 5);


// Maximum number of recursive calls
const int max_bounce = 5;


// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;


// Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.5, 0.1, 0.1, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);
const Vector4d obj_refraction_color(0.7, 0.7, 0.7, 0);


// Precomputed gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;


// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
// Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);


// Fills the different arrays
void setup_scene() {
    grid.resize(grid_size + 1);
    for (int i = 0; i < grid_size + 1; ++i)
    {
        grid[i].resize(grid_size + 1);
        for (int j = 0; j < grid_size + 1; ++j)
            grid[i][j] = Vector2d::Random().normalized();
    }

    // Spheres
    sphere_centers.emplace_back(10, 0, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(7, 0.05, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(4, 0.1, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-2, 0.4, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-5, 0.8, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-8, 1.6, 1);
    sphere_radii.emplace_back(1);

    // Parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    // Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}


// We need to make this function visible
Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce);


////////////////////////////////////////////////////////////////////////////////
// Perlin noise code
////////////////////////////////////////////////////////////////////////////////


// Function to linearly interpolate between a0 and a1
// Weight w should be in the range [0.0, 1.0]
double lerp(double a0, double a1, double w) {
    assert(w >= 0);
    assert(w <= 1);

    // Uncomment these lines for linear interpolation
    // double linearInterpolation = (1.0 - w)*a0 + w*a1;
    // return linearInterpolation;

    // Uncomment these lines for cubic interpolation
    double cubicInterpolation = (a1 - a0) * (3.0 - w * 2.0) * w * w + a0;
    return cubicInterpolation;
}


// Computes the dot product of the distance and gradient vectors.
double dotGridGradient(int ix, int iy, double x, double y) {
    // Compute the distance vector
    double dx = x - (double) ix;
    double dy = y - (double) iy;

    // Compute the dot-product
    double dotProduct = dx*grid[iy][ix][0] + dy*grid[iy][ix][1];

    return dotProduct;
}


// Compute Perlin noise at coordinates x, y
double perlin(double x, double y) {
    // Determine grid cell coordinates x0, y0
    int x0 = int(x);
    int x1 = x0 + 1;
    int y0 = int(y);
    int y1 = y0 + 1;

    // Determine interpolation weights
    double sx = x - (double) x0;
    double sy = y - (double) y0;


    // Interpolate between grid point gradients
    double n0, n1, ix0, ix1, value;

    n0 = dotGridGradient(x0, y0, x, y);
    n1 = dotGridGradient(x1, y0, x, y);
    ix0 = lerp(n0, n1, sx);

    n0 = dotGridGradient(x0, y1, x, y);
    n1 = dotGridGradient(x1, y1, x, y);
    ix1 = lerp(n0, n1, sx);

    value = lerp(ix0, ix1, sy);

    return value;
}

// Generate procedural textures with checkerboard pattern or perlin noise
Vector4d procedural_texture(const double tu, const double tv) {
    assert(tu >= 0);
    assert(tv >= 0);
    assert(tu <= 1);
    assert(tv <= 1);


    // Uncomment these lines for perlin noise
    const double color = (perlin(tu * grid_size, tv * grid_size) + 1) / 2;
    return Vector4d(0, color, 0, 0);

    // Uncomment these lines for checkerboard pattern
    // const double color = (int(tu * grid_size) + int(tv * grid_size)) % 2 == 0 ? 0 : 1;
    // return Vector4d(0, color, 0, 0);
}


////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////


// Compute the intersection between a ray and a sphere, return t or -1 for no intersection
double ray_sphere_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N) {
    // Sphere Center
    const Vector3d sphere_center = sphere_centers[index];
    // Sphere Radius
    const double sphere_radius = sphere_radii[index];


    // Terms for solving the discriminant
    const double A = ray_direction.dot(ray_direction);
    const double B = ray_direction.dot(ray_origin - sphere_center);
    const double C =  (ray_origin - sphere_center).dot(ray_origin - sphere_center) - sphere_radius*sphere_radius;
   
    // Solution from textbook (Introduction to Computer Graphics, 5th edition, Chapter 4, Ray-Sphere Intersection)
    const double discriminant = B*B - A*C;

    // No intersection
    if (discriminant < 0)
    {
        return -1;
    }


    // 2 possible solutions for t: take smallest/closest
    double t = (-ray_direction.dot(ray_origin - sphere_center)-sqrt(discriminant))/(ray_direction.dot(ray_direction));


    // Intersection point
    p = ray_origin + t*ray_direction;
    // Normalized normal at intersection point
    N = ((p - sphere_center)/sphere_radius).normalized();

    // NOTE: p and N are returned as pointers 
    return t;
}


// Compute the intersection between a ray and a paralleogram, return t or -1 for no intersection
double ray_parallelogram_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N) {
    // Bottom left corner of parallelogram
    const Vector3d pgram_origin = parallelograms[index].col(0);
    // Points for corners of parallelogram
    const Vector3d pgram_p1 = parallelograms[index].col(1);
    const Vector3d pgram_p2 = parallelograms[index].col(2);
    // Vectors from pgram origin to respective corners
    const Vector3d pgram_u = pgram_p1 - pgram_origin;
    const Vector3d pgram_v = pgram_p2 - pgram_origin;


    // Matrix for Cramer's Rule
    Matrix3d A;
    // Load values
    A << pgram_u, pgram_v, -ray_direction;


    // Result vector
    Vector3d b = ray_origin - pgram_origin;
    // Solve for unknowns [u, v, t]
    Vector3d x = A.inverse()*b;


    double u = x(0);
    double v = x(1);
    double t = x(2);


    // Ray intersects
    if (t > 0 && u >= 0 &&  u <= 1  && v >= 0 && v <= 1)
    {
        // Intersection point
        p = ray_origin + t*ray_direction;
        // Normalized normal at intersection point
        N = ((pgram_v).cross(pgram_u)).normalized();
        return t;
    }


    // Ray does not intersect
    return -1;
}


// Finds the closest intersecting object; returns its index
int find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N) {
    // Array index of the closest object intersected
    int closest_index = -1;
    // Closest intermediate intersection; initiallized to positive infinity
    double closest_t = std::numeric_limits<double>::max();

    // Intermediate p and N
    Vector3d tmp_p, tmp_N;

    // Loop through spheres in scene 
    for (int i = 0; i < sphere_centers.size(); ++i) {
        // Returns t and sets tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);

        // We have intersection
        if (t >= 0) {
            // Intersected sphere is closer than the previous sphere
            if (t < closest_t) {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    // Loop through planes in scene (Only 1 for this case)
    for (int i = 0; i < parallelograms.size(); ++i) {
        // Returns t and sets tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);

        // We have intersection
        if (t >= 0) {
            // Intersected plane is closer than the previous plane
            if (t < closest_t) {
                closest_index = sphere_centers.size() + i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    // NOTE: if a plane is intersected, the index is an overrun of the amount of spheres in the scene 
    // (i.e. plane_matrix = parallelograms(sphere_centers.size() - closest_index)) 
    return closest_index;
}


////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////


// Determines if the light is visible for a point
bool is_light_visible(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &light_position) {
    // Point to evaluate for lighting, and surface nornmal
    Vector3d p, N;
    // Index of closest object in ray's path
    const int nearest_objectI = find_nearest_object(ray_origin, ray_direction, p, N);


    // Define shadow ray offset, origin and direction
    Vector3d shadow_ray_origin, shadow_ray_direction;
    double epsilon = 1e-5;

    // Unit vector from intersection point to light position
    shadow_ray_direction = (light_position - p).normalized();
    // Offset shadow ray to avoid self-collisions
    shadow_ray_origin = p + epsilon*shadow_ray_direction;


    // Unused in this case, but needed for calling find_nearest_object() 
    Vector3d shadowP, shadowN;

    // Find closest object in shadow ray's path
    const int shadow_nearest_objectI = find_nearest_object(shadow_ray_origin, shadow_ray_direction, shadowP, shadowN);


    // Point is in shadow
    if(shadow_nearest_objectI > 0){
        return false;
    }

    // Point is in light
    return true;
}


Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce) {
    // Intersection point and normal; output of find_nearest_object
    Vector3d p, N;

    // Index of nearest object in object arrays
    const int nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);


    if (nearest_object < 0) {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }
   
    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();


    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    // Loop through every light in the scene
    for (int i = 0; i < light_positions.size(); ++i) {
        // Current light
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];


        // Unit vector from intersection point to light position
        const Vector3d Li = (light_position - p).normalized();
        // Unit vector towards camera
        const Vector3d v = -ray_direction.normalized();
        // Bisector of the angle between Li and v
        const Vector3d h = (v + Li).normalized();


        // Delcared in this scope for more efficient reflections
        Vector4d specular(0,0,0,0);
        // Shoots a shadow ray to determine if the light should affect the intersection point
        if(is_light_visible(ray_origin, ray_direction, light_position)) {
            Vector4d diff_color = obj_diffuse_color;

            // Shading for procedurally textured sphere (specific case)
            if (nearest_object == 4) {
                // Compute UV coodinates for the point on the sphere
                const double x = p(0) - sphere_centers[nearest_object][0];
                const double y = p(1) - sphere_centers[nearest_object][1];
                const double z = p(2) - sphere_centers[nearest_object][2];
                const double tu = acos(z / sphere_radii[nearest_object]) / 3.1415;
                const double tv = (3.1415 + atan2(y, x)) / (2 * 3.1415);


                diff_color = procedural_texture(tu, tv);
            }


            // Diffuse contribution
            const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);
            // Specular contribution
            specular = obj_specular_color * std::max(pow(N.dot(h),obj_specular_exponent), 0.0);

            // Attenuate lights according to the squared distance to the lights
            const Vector3d D = light_position - p;
            // Sum direct contributions from each light
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }
    }


    Vector4d refl_color = obj_reflection_color;
    if (nearest_object == 4) {
        refl_color = Vector4d(0.5, 0.5, 0.5, 0);
    }

    // Compute the color of the reflected ray recursively 
    Vector4d reflection_color(0,0,0,0);
    if (max_bounce > 0) {
        // Ray offset
        double epsilon = 1e-5;
        // Unit vector towards camera
        const Vector3d v = -ray_direction.normalized();
        // Reflection direction
        Vector3d reflection_ray_direction = 2*N*(N.dot(v))-v;
        // Offset reflection ray origin to avoid self collisions
        Vector3d reflection_ray_origin = p + epsilon*reflection_ray_direction;

        // Shoot ray from intersection point towards reflection direction
        reflection_color = shoot_ray(reflection_ray_origin, reflection_ray_direction, max_bounce-1);
    }

    // TODO: Compute the color of the refracted ray and add its contribution to the current point color.
    // Make sure to check for total internal reflection before shooting a new ray.
    Vector4d refraction_color(0, 0, 0, 0);


    // Sum color contributions 
    Vector4d C = ambient_color + lights_color + refl_color.cwiseProduct(reflection_color);


    // Set alpha to 1
    C(3) = 1;


    return C;
}


////////////////////////////////////////////////////////////////////////////////


void raytrace_scene() {
    std::cout << "Ray Tracing Scene..." << std::endl;

    // Height and width of image
    int w = 800;
    int h = 400;
    // RGBA matrices
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask


    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers a viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    // Trivial (i.e. FOV is given as vertical FOV)
    const double fov_vertical = field_of_view;
    // Calculate horizontal FOV using aspect ratio (1:2) and vertical FOV
    const double fov_horizontal = 2.0 * atan(aspect_ratio * tan(fov_vertical / 2.0));

    // Compute correct pixel size
    double image_y = 2.0 * image_z * tan(fov_vertical / 2.0);
    double image_x = 2.0 * image_z * tan(fov_horizontal / 2.0);


    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, -image_z);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);


    // Loop through each pixel in the scene
    for (unsigned i = 0; i < w; ++i) {
        for (unsigned j = 0; j < h; ++j) {
            // TODO: Implement depth of field
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective) {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = pixel_center - camera_position;
            }
            else {
                // Orthographic camera
                ray_origin = camera_position + Vector3d(pixel_center[0], pixel_center[1], 0);
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce);

            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}


////////////////////////////////////////////////////////////////////////////////


int main(int argc, char *argv[]) {
    setup_scene();
    raytrace_scene();

    return 0;
}