import meshlib.mrmeshpy as mr
import os

def create_mesh(file_name, output_file_name):
    # load image as Distance Map object:
    dm = mr.loadDistanceMapFromImage(mr.Path(file_name), 0)
    # find boundary contour of the letter:
    polyline2 = mr.distanceMapTo2DIsoPolyline(dm, isoValue=127)
    # triangulate the contour
    # Assuming you have an instance of HolesVertIds
    holes_vert_ids = mr.HolesVertIds()

    # Now call contours2 with the required argument
    mesh = mr.triangulateContours(polyline2.contours2(holes_vert_ids))
    # mesh = mr.triangulateContours(polyline2.contours2())
    # extrude itself:
    mr.addBaseToPlanarMesh(mesh, zOffset=30)
    # export the result:
    mr.saveMesh(mesh, mr.Path(output_file_name))

def get_all_files(directory):
    # List all files and directories in the given directory
    all_entries = os.listdir(directory)
    # Filter out only the files
    files = [entry for entry in all_entries if os.path.isfile(os.path.join(directory, entry))]
    return files

if __name__=="__main__":
    # get all files in the 2d_maps/2d_map_images folder
    files = []

    # Example usage
    directory_path = '2d_maps/images'
    output_dir_path = '2d_maps/meshes'
    files = get_all_files(directory_path)

    # for each file, generate the sdf
    for file in files:
        mesh_name = file.split('.')[0]
        create_mesh(f"{directory_path}/{file}", f"{output_dir_path}/{mesh_name}.stl")