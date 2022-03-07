#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <set>
#include <fstream>
#include <sstream>
#include <cctype>
#include <utility>
#include <string_view>



std::string getText(const char* file) {
    std::ifstream in(file);
    if (!in) {
        throw "error opening file";
    }
    std::string data;
    data.assign(std::istreambuf_iterator<char>(in.rdbuf()), std::istreambuf_iterator<char>());
    for (uint64_t i = 0; i < data.size(); i++) {
        if (data[i] >= 'A' && data[i] <= 'Z') {
            data[i] = tolower(data[i]);
        } else if (data[i] < 'a' || data[i] > 'z') {
            data[i] = ' ';
        }
    }
    return data;
}

std::vector<std::pair<std::string_view, uint64_t>> getSortedVector(const std::map<std::string, uint64_t>& dict) {
    std::vector<std::pair<std::string_view, uint64_t>> res;
    for (const auto& [key, val] : dict) {
        res.emplace_back(key, val);
    }

    auto cmp = [] (const std::pair<std::string_view, uint64_t>& a, const std::pair<std::string_view, uint64_t>& b) {
        return (a.second == b.second) ? a.first < b.first : a.second > b.second;
    };
    std::sort(res.begin(), res.end(), cmp);
    return res;
}


int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Usage:\n";
        std::cout << "./freq inputFile outputFile" << std::endl;
        return 0;
    }
    
    std::map<std::string, uint64_t> freqDict;
    {
        std::string data = getText(argv[1]);
        std::stringstream input(data);

        std::string word;
        while (input >> word) {
            freqDict[word]++;
        }
    }

    std::vector<std::pair<std::string_view, uint64_t>> res = getSortedVector(freqDict);
    std::ofstream output(argv[2]);
    for (auto& i : res) {
        output << i.second << " " << i.first << std::endl;
    }
    return 0;
}
