
#include "mess2_logger/msg_parser.hpp"
#include <iostream>

std::string get_msg_file_path(
    const std::string &msg_type
) {
    size_t pos = msg_type.find("/");
    if (pos == std::string::npos) {
        throw std::invalid_argument("Encountered unexpected message type : " + msg_type);
    }

    std::string ss1 = msg_type.substr(0, pos);
    std::string ss3 = msg_type.substr(pos + 1);
    std::string share_dir_path = ament_index_cpp::get_package_share_directory(ss1);
    return share_dir_path + "/" + ss3 + ".msg";
}


bool is_field_type_array(
    const std::string &field_type,
    std::smatch &match
) {
    return std::regex_match(field_type, match, ARRAY_REGEX);
}


bool is_field_type_primitive(
    const std::string &field_type
) {
    return PRIMATIVES_SET.find(field_type) != PRIMATIVES_SET.end();
}


std::vector<std::string> parse_msg(
    const std::string &prefix,
    const std::string &msg_type
) {
    std::vector<std::string> msg_fields;

    std::string msg_file_path = get_msg_file_path(msg_type);
    std::ifstream file(msg_file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file : " + msg_file_path);
    }

    std::string line;
    while (std::getline(file, line)) {
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::string field_type, field_name;
        iss >> field_type >> field_name;

        std::smatch match;
        if (is_field_type_array(field_type, match)) {
            field_type = match[1];
            std::string field_size = match[2];
        }

        if (is_field_type_primitive(field_type)) {
            msg_fields.push_back(prefix + "." + field_name);
        } else {
            std::string nested_prefix = prefix + "." + field_name;
            std::string nested_type;

            if (field_type.find("/") != std::string::npos) {
                size_t pos = field_type.find("/");
                std::string ss1 = field_type.substr(0, pos);
                std::string ss3 = field_type.substr(pos + 1);
                if (ss3.find("msg/") == std::string::npos) {
                    ss3 = "msg/" + ss3;
                }
                nested_type = ss1 + "/" + ss3;
            } else {
                const std::string search = "/share/";
                size_t pos = msg_file_path.find(search);
                if (pos == std::string::npos) {
                    throw std::invalid_argument("Msg file path does not contain '/share/' : " + msg_file_path);
                }

                size_t pos1 = pos + search.length();
                size_t pos2 = msg_file_path.find("/", pos1);
                if (pos2 == std::string::npos) {
                    throw std::invalid_argument("Unexpected path structure after '/share/' : " + msg_file_path);
                }

                std::string ss1 = msg_file_path.substr(pos1, pos2 - pos1);
                std::string ss3 = field_type;
                if (ss3.find("msg/") == std::string::npos) {
                    ss3 = "msg/" + ss3;
                }
                nested_type = ss1 + "/" + ss3;
            }

            auto nested_fields = parse_msg(nested_prefix, nested_type);
            msg_fields.insert(msg_fields.end(), nested_fields.begin(), nested_fields.end());
        }
    }

    return msg_fields;
}


PYBIND11_MODULE(msg_parser, m) {
    m.doc() = "ros2 message parser";
    m.def("parse_msg", &parse_msg, "Parses a ROS2 message file from its share directory to obtain a list of all field names."
    );
}
