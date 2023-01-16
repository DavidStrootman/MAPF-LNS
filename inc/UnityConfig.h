#include <vector>
#include <array>
#include <cassert>
#include <string>

#include <boost/json.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace property_tree = boost::property_tree;


struct UnityConfig {
    std::vector<std::vector<bool>> matrix;
    std::vector<std::array<int, 2>> initialNodes;
    std::vector<std::array<int, 2>> goalNodes;
    int timeLimit;
    string outputFile;
    string statsFile;
    string solver;
    string initAlgorithm;
    string replanAlgorithm;

    UnityConfig(std::vector<std::vector<bool>> matrix,
                std::vector<std::array<int, 2>> initialNodes,
                std::vector<std::array<int, 2>> goalNodes,
                int timeLimit,
                string outputFile,
                string statsFile,
                string solver,
                string initAlgorithm,
                string replanAlgorithm) :
            matrix(matrix),
            initialNodes(initialNodes),
            goalNodes(goalNodes),
            timeLimit(timeLimit),
            outputFile(outputFile),
            statsFile(statsFile),
            solver(solver),
            initAlgorithm(initAlgorithm),
            replanAlgorithm(replanAlgorithm)
            {
        assert (initialNodes.size() == goalNodes.size());
    };

    static std::vector<std::array<int, 2>> parseNodesByKey(property_tree::ptree ptree, std::string key) {
        property_tree::ptree nodes_tree = ptree.get_child(key);
        std::vector<std::array<int, 2>> nodes = {};
        std::array<int, 2> node = {};
        size_t position_index = 0;

        // Iterate over all nodes
        for (property_tree::ptree::iterator nodes_iterator = nodes_tree.begin();
             nodes_iterator != nodes_tree.end();
             nodes_iterator++) {

            // Iterate over xy pairs
            for (property_tree::ptree::iterator position_xy_iterator = nodes_iterator->second.get_child(
                    "grid_position").begin();
                 position_xy_iterator != nodes_iterator->second.get_child("grid_position").end();
                 ++position_xy_iterator) {
                node[position_index] = position_xy_iterator->second.get_value<int>();
                ++position_index;
            }
            // Push newly created node
            nodes.push_back(node);
            // Reset node and position iterator
            node = {};
            position_index = 0;
        }

        return nodes;
    }

    static std::vector<std::array<int, 2>> parseInitialNodes(property_tree::ptree ptree) {
        return UnityConfig::parseNodesByKey(ptree, "start_nodes");
    }


    static std::vector<std::array<int, 2>> parseGoalNodes(property_tree::ptree ptree) {
        return UnityConfig::parseNodesByKey(ptree, "goal_nodes");
    }

    static std::vector<std::vector<bool>> parseMapMatrix(property_tree::ptree ptree) {
        std::vector<std::vector<bool>> mapMatrix = {};
        property_tree::ptree map_tree = ptree.get_child("matrix.matrix");
        size_t x = 0;

        for (property_tree::ptree::iterator line_iterator = map_tree.begin();
             line_iterator != map_tree.end();
             line_iterator++) {
            mapMatrix.push_back(std::vector<bool>());
            for (property_tree::ptree::iterator position_iterator = line_iterator->second.begin();
                 position_iterator != line_iterator->second.end();
                 ++position_iterator) {
                mapMatrix[x].push_back(position_iterator->second.get_value<bool>());
            }
            ++x;
        }
        return mapMatrix;
    }

    static UnityConfig fromPTree(property_tree::ptree ptree) {
        std::vector<std::vector<bool>> matrix = parseMapMatrix(ptree);
        std::vector<std::array<int, 2>> initialNodes = parseInitialNodes(ptree);
        std::vector<std::array<int, 2>> goalNodes = parseGoalNodes(ptree);
        int timeLimit = ptree.get<int>("time_limit");
        string outputFile = ptree.get<string>("output_file");
        string statsFile = ptree.get<string>("stats_file");
        string solver = ptree.get<string>("solver_algorithm");
        string initAlgorithm = ptree.get<string>("init_algorithm");
        string replanAlgorithm = ptree.get<string>("replan_algorithm");

        return UnityConfig(matrix, initialNodes, goalNodes, timeLimit, outputFile, statsFile, solver, initAlgorithm, replanAlgorithm);
    }

    static UnityConfig fromJSONFile(string filePath) {
        std::ifstream jsonFile(filePath);
        std::stringstream buffer;
        // Read entire file contents into file buffer, so we can parse it.
        buffer << jsonFile.rdbuf();

        // Read the buffer containing the EnvValues into a property tree.
        property_tree::ptree jsonContents;
        property_tree::read_json(buffer, jsonContents);
        // Parse the property tree into a EnvValues object
        return fromPTree(jsonContents);
    };
};