#include "gt_reader.h"
#include <stdexcept>

std::vector<std::vector<std::vector<int>>> bbreader(std::string dir, int tray) {
    if(tray<1 || tray>8)
        throw std::invalid_argument("Argument tray must be in range 1-8 included");
    std::string PATH = dir+"tray"+std::to_string(tray)+"/bounding_boxes/";
    
    std::vector<std::string> files;
    files.push_back("food_image_bounding_box.txt");
    for(int i=0; i<2; i++)
        files.push_back("leftover"+std::to_string(i+1)+"_bounding_box.txt");

    std::vector< std::vector< std::vector<int> > > filecontainer;
    for(int i=0; i<files.size(); i++) {
        std::vector< std::vector<int> > file_v;
        std::string line;
        std::fstream readfile;
        std::string pathfile = PATH+files[i];
        
        readfile.open(pathfile);

        while(std::getline(readfile, line)) {
            std::vector<int> line_v;
            // removing spaces
            line.erase(remove(line.begin(), line.end(), ' '), line.end());
            // extract class
            int j=0;
            while(line[j]!=';')
                j++;
            line_v.push_back(std::stoi(line.substr(3, j-2)));
            // extract rectangle
            std::string coordinates = line.substr(j+2, line.size()-j-3);
            int comma[3];
            int l=0;
            for(int k=0; k<coordinates.size(); k++) {
                if(coordinates[k]==',') {
                    comma[l]=k;
                    l++;
                }
            }
            line_v.push_back(std::stoi(coordinates.substr(0, comma[0])));
            line_v.push_back(std::stoi(coordinates.substr(comma[0]+1, comma[1]-comma[0]-1)));
            line_v.push_back(std::stoi(coordinates.substr(comma[1]+1, comma[2]-comma[1]-1)));
            line_v.push_back(std::stoi(coordinates.substr(comma[2]+1, coordinates.size()-comma[1]-1)));
            // insert line in file coordinates vector
            file_v.push_back(line_v);
        }
        readfile.close();
        // insert file coordinates in tray coordinates vector
        filecontainer.push_back(file_v);
    }
    return filecontainer;
}