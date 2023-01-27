//
// Created by zzy on 3/14/18.
//
#include <omp.h>
#include <ctime>
#include <vector>
#include <string>
#include <dirent.h>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

#include <sys/stat.h>

#include <pcl/io/boost.h>
#include <boost/program_options.hpp>

#include <fstream>


//namespace fs = std::experimental::filesystem::v1;

static std::vector<std::string> file_lists;


class CommandLineArgs
{
public:
    CommandLineArgs(int argc, char **argv);
    ~CommandLineArgs(){}
    bool process_command_line(
        int argc, 
        char** argv);

    std::string _bin_path;
    std::string _pcd_path;
    std::string _mode;
private:
    CommandLineArgs(){}
    boost::program_options::options_description *_desc;  


};

CommandLineArgs::CommandLineArgs(int argc, char **argv)
{
    _bin_path = "/home/kitti_velodyne_bin_to_pcd/bin/";
    _pcd_path = "/home/kitti_velodyne_bin_to_pcd/pcd/";

    _desc = new boost::program_options::options_description("Program Usage", 1024, 512);
    _desc->add_options()
                    ("help",     "produce help message")
                    ("b",   boost::program_options::value<std::string>(&_bin_path)->required(), "bin file folder")
                    ("p",   boost::program_options::value<std::string>(&_pcd_path)->required(), "pcd file folder")
                    ("m",   boost::program_options::value<std::string>(&_mode)->required(),     "mode - bin2pcd, pcd2bin")                    
                    ;
    process_command_line(argc, argv);
}

bool CommandLineArgs::process_command_line(
        int argc, 
        char** argv)
{
    try
    {

        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, *_desc), vm);

        if (vm.count("help"))
        {
            std::cout << *_desc << "\n";
            return false;
        }

        // There must be an easy way to handle the relationship between the
        // option "help" and "host"-"port"-"config"
        // Yes, the magic is putting the po::notify after "help" option check
        boost::program_options::notify(vm);
    }
    catch (std::exception &e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown error!"
                  << "\n";
        return false;
    }
    return true;
}

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

void readKittiPclBinData(std::string &in_file, std::string& out_file)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, true);
}


void convertPCDtoBin(std::string &in_file, std::string& out_file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(in_file, *cloud) == -1) //* load the file
    {
        std::string err = "Couldn't read file " + in_file;
        PCL_ERROR(err.c_str());
        return;// (-1);
    }
    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from " 
            << in_file
            << " with the following fields: "
            << std::endl;

    int data_idx = 0;
    std::ostringstream oss;
    oss << pcl::PCDWriter::generateHeader(*cloud);// << "DATA binary\n";
    oss.flush();
    data_idx = static_cast<int>(oss.tellp());

    std::vector<pcl::PCLPointField> fields;
    std::vector<int> fields_sizes;
    size_t fsize = 0;
    size_t data_size = 0;
    size_t nri = 0;
    pcl::getFields (*cloud, fields);

    // Compute the total size of the fields
    for (const auto &field : fields)
    {
        if (field.name == "_")
        {
            continue;
        }
        
        int fs = field.count * pcl::getFieldSize (field.datatype);
        fsize += fs;
        fields_sizes.push_back (fs);
        fields[nri++] = field;
    }
    fields.resize (nri);

    data_size = cloud->points.size () * fsize;
    const int memsize = cloud->points.size() * sizeof(float) * 4;
    char *out = (char*)malloc( memsize);// 4 field x y z intensity
    std::cout << "data_size size: " << data_size << std::endl;
    // char buffer[100];
    std::ofstream myFile (out_file.c_str(), std::ios::out | std::ios::binary);
    

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        int nrj = 0;
        for (const auto &field : fields)
        {
            memcpy(out, reinterpret_cast<const char*> (&cloud->points[i]) + field.offset, fields_sizes[nrj++]);
            //myFile.write (reinterpret_cast<const char*> (&cloud->points[i]) + field.offset, fields_sizes[nrj++]);
        }
        float intensity = 0;
        memcpy(out, reinterpret_cast<const char*> (&intensity) , sizeof(intensity));
        // myFile.write ( reinterpret_cast<const char*> (&intensity) , sizeof(intensity));
    }
    myFile.write(out, memsize);

    myFile.close();
}

void updateRear(std::string &pathStr)
{
    if (pathStr != "" && pathStr.back() != '/')
    {
        pathStr += "/";
    }
}

int main(int argc, char **argv)
{
    
    CommandLineArgs cmd_args(argc, argv);

    // Create _outputFile folder if not exist
    struct stat sb;
    std::string folderPath = cmd_args._pcd_path;
    if (! (stat(folderPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) )
    {//...It is not a directory...
        mkdir(folderPath.c_str(), 0755);
    }
    folderPath = cmd_args._bin_path;
    if (! (stat(folderPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) )
    {//...It is not a directory...
        mkdir(folderPath.c_str(), 0755);
    }

    

    if(cmd_args._mode == "bin2pcd")
    {
        read_filelists( cmd_args._bin_path, file_lists, "bin" );
        sort_filelists( file_lists, "bin" );

        #pragma omp parallel num_threads(8)
        #pragma omp parallel for
        for (int i = 0; i < file_lists.size(); ++i)
        {
            std::string bin_file = cmd_args._bin_path + file_lists[i];
            std::string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".pcd";
            std::string pcd_file = cmd_args._pcd_path + tmp_str;
            readKittiPclBinData( bin_file, pcd_file );
        }
    } 
    else if(cmd_args._mode == "pcd2bin")
    {
        read_filelists( cmd_args._pcd_path, file_lists, "pcd" );
        sort_filelists( file_lists, "pcd" );

        std::cout << "Run pcd2bin" << std::endl;
        #pragma omp parallel num_threads(8)
        #pragma omp parallel for
        for (int i = 0; i < file_lists.size(); ++i)
        {
            std::string pcd_file = cmd_args._pcd_path + file_lists[i];
            std::string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".bin";
            std::string bin_file = cmd_args._bin_path + tmp_str;
            std::cout << pcd_file << "\n"
                      << bin_file << std::endl;
            convertPCDtoBin( pcd_file, bin_file );
        }

    }
    else
    {
        std::cout << "No mode provided" << std::endl;
    }
    

    

    return 0;
}