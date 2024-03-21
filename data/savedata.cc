#include "savedata.h"

void saveToCSV(const std::vector<double>& data, const std::string& filename) {
    // 打开文件
    std::ofstream outputFile("../data/" + filename);

    // 检查文件是否成功打开
    if (!outputFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 写入数据到文件
    for (const double& value : data) {
        outputFile << value << '\n';
    }

    // 关闭文件
    outputFile.close();

    std::cout << "数据已保存到 ../data/" << filename << std::endl;
}


void saveLaneToCSV(const std::vector<point>& data, const std::string& filename){
    // 打开文件
    std::ofstream outputFile("../data/" + filename);

    // 检查文件是否成功打开
    if (!outputFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 写入数据到文件
    for (int i = 0; i < data.size(); i++){
        outputFile << data[i].x << ',' << data[i].y << '\n';
    }

    // 关闭文件
    outputFile.close();

    std::cout << "数据已保存到 ../data/" << filename << std::endl;
}