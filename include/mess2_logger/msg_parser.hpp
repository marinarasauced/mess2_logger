#ifndef MESS2_LOGGER_CPP_MSG_PARSER_HPP
#define MESS2_LOGGER_CPP_MSG_PARSER_HPP

#include <filesystem>
#include <fstream>
#include <regex>
#include <unordered_set>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ament_index_cpp/get_package_share_directory.hpp"


//
static const std::regex ARRAY_REGEX(R"(([\w/]+)\[(\d*)\])");
static const std::unordered_set<std::string> PRIMATIVES_SET = {
    "bool", "byte", "char", "float32", "float64",
    "int8", "uint8", "int16", "uint16", "int32", "uint32",
    "int64", "uint64", "string", "wstring"
};



/**
 * 
 */
std::string get_msg_file_path(
    const std::string &msg_type
);


/**
 * 
 */
bool is_field_type_array(
    const std::string &field_type,
    std::smatch &match
);


/**
 * 
 */
bool is_field_type_primitive(
    const std::string &field_type
);


/**
 * 
 */
std::vector<std::string> parse_msg(
    const std::string &prefix,
    const std::string &msg_type
);


#endif // MESS2_LOGGER_CPP_MSG_PARSER_HPP
