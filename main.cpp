#include <igl/read_triangle_mesh.h>
#include <igl/hausdorff.h>
#include <cinolib/meshes/trimesh.h>
#include <cinolib/string_utilities.h>

std::streambuf* old_buf = std::cout.rdbuf();
void silence_output() {
    std::streambuf* null_buf = nullptr;
    std::ostream null_stream(null_buf);
    std::cout.rdbuf(null_stream.rdbuf());
}
void restore_output() {
    std::cout.rdbuf(old_buf);
}

int main(int argc, char *argv[]) {

    if (argc < 3) {
        std::cout << "Computes the Hausdorff distance between two meshes." << std::endl;
        std::cout << "Usage: " << argv[0] << " <meshA.obj> <meshB.obj>" << std::endl;
        std::cout << "Output: <meshA_name>; <hausdorff_distance>; <hausdorf_distance / meshA_bbox_min_xyz>" << std::endl;
        return 1;
    }

    // Load the meshes
    Eigen::MatrixXd VA, VB;
    Eigen::MatrixXi FA, FB;
    igl::read_triangle_mesh(argv[1], VA, FA);
    igl::read_triangle_mesh(argv[2], VB, FB);

    // Compute the Hausdorff distance
    double hausdorff_dist;
    igl::hausdorff(VA, FA, VB, FB, hausdorff_dist);

    // Load the first mesh using cinolib to compute its bounding box
    silence_output();
    cinolib::Trimesh<> mesh(argv[1]);
    restore_output();
    double dim = std::min(mesh.bbox().delta_x(), std::min(mesh.bbox().delta_y(), mesh.bbox().delta_z()));

    // Output the results
    std::cout << cinolib::get_file_name(argv[1], false) << "; " << hausdorff_dist << "; " << hausdorff_dist / dim << std::endl;

    return 0;
}