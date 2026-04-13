#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib> // 用於 std::getenv
#include <cmath>

struct Point {
    double x;
    double y;
};

int main() {
    // 1. 設定檔案路徑
    std::string inputFileName = "/home/cyc/20260123v2.csv";
    
    // 獲取 HOME 目錄路徑
    const char* homeDir = std::getenv("HOME");
    if (!homeDir) {
        std::cerr << "錯誤：找不到 HOME 環境變數！" << std::endl;
        return 1;
    }
    std::string outputFileName = std::string(homeDir) + "/points.csv";

    // 2. 開啟檔案
    std::ifstream inFile(inputFileName);
    if (!inFile.is_open()) {
        std::cerr << "無法開啟輸入檔案: " << inputFileName << std::endl;
        return 1;
    }

    std::ofstream ofs_back(outputFileName);
    if (!ofs_back.is_open()) {
        std::cerr << "無法開啟 " << outputFileName << "！" << std::endl;
        return 1;
    }

    double angle_deg = -8.0; // 設定旋轉角度 (度)
    double angle_rad = angle_deg * (M_PI / 180.0); // 轉換為弧度
    double cos_theta = std::cos(angle_rad);
    double sin_theta = std::sin(angle_rad);
    // 3. 處理資料
    std::string line;
    // 寫入 CSV 標題
    ofs_back << "x,y\n";

    // 跳過輸入檔案的第一行標題
    std::getline(inFile, line);

    int count = 0;
    while (std::getline(inFile, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string xStr, yStr;

        if (std::getline(ss, xStr, ',') && std::getline(ss, yStr, ',')) {
            try {
                double old_x = std::stod(xStr);
                double old_y = std::stod(yStr);

                // --- 這裡套用你要求的轉換邏輯 ---
                // double x_rot1 =  -p.y - 4.8;
                // double y_rot2 =  p.x + 43.8;
                double new_x = -old_x +30;
                double new_y = -old_y -70 ;
                // ----------------------------
                new_x = new_x * cos_theta - new_y * sin_theta;
                new_y = new_x * sin_theta + new_y * cos_theta;

                new_x = new_x +4;
                new_y = new_y -12;

                ofs_back << new_x << "," << new_y << "\n";
                count++;
            } catch (const std::exception& e) {
                // 忽略無法轉換為數字的行
            }
        }
    }

    // 4. 關閉並輸出結果
    inFile.close();
    ofs_back.close();  

    std::cout << "已成功處理 " << count << " 個座標點。" << std::endl;
    std::cout << "結果已儲存至：" << outputFileName << std::endl;

    return 0;
}