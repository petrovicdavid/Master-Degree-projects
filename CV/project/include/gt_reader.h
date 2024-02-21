#ifndef __GT_READER_H__
#define __GT_READER_H__

#include <vector>
#include <string>
#include <fstream>
#include <algorithm>

/**
 * Reader for GT bounding boxes
 * Returns all bounding boxes for specified tray
 * 
 * dir      path of parent directory (where tray directories stay)
 * tray     number of the tray [1,8]
*/
std::vector<std::vector<std::vector<int>>> bbreader(std::string dir, int tray);

#endif