//  Copyright 2019 Tsung-Han Brian Lee
/*********************************************************
 ** -------------- A* Algorithms -----------------
 **   AUTHOR      : Tsung-Han Brian Lee
 **   DESCRIPTION :
 ** -----------------------------------------------
 *********************************************************/
#ifndef _A_STAR_
#define _A_STAR_
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>
#include <string>

/**********************************************************************
 ** Node to represent a node in space
 **********************************************************************/
class Node {
 public:
    friend std::ostream& operator << (std::ostream& out, const Node& n);

    Node(int x, int y) : _x(x), _y(y) {}
    Node(const Node& node) : _x(node._x), _y(node._y) {}
    Node()  : _x(0), _y(0) {}

    int getX() const {return _x;}
    int getY() const {return _y;}
    void walk(const int xDiff, const int yDiff) {
        _x += xDiff;
        _y += yDiff;
    }

 protected:
    int _x;
    int _y;
};

std::ostream& operator << (std::ostream& out, const Node& n) {
    out << "(" << n.getX() << ", " << n.getY() << ")";
    return out;
}

/**********************************************************************
 ** MapNode to represent a node in map
 **********************************************************************/

class MapNode : public Node {
 public:
    friend bool operator < (const MapNode& lhs, const MapNode& rhs);
    friend bool operator > (const MapNode& lhs, const MapNode& rhs);
    friend bool operator == (const MapNode& lhs, const MapNode& rhs);
    friend bool operator <= (const MapNode& lhs, const MapNode& rhs);
    friend bool operator >= (const MapNode& lhs, const MapNode& rhs);
    friend std::ostream& operator << (std::ostream& out, const MapNode& n);
    friend class OpenList;

    MapNode() : Node(0, 0) {
         parent = NULL;
     }
    MapNode(int x, int y) : Node(x, y) {
        parent = NULL;
    }
    MapNode(int x, int y, MapNode* parentPtr) : Node(x, y) {
        parent = parentPtr;
    }

    double getGCost() const {return gCost;}
    double getHCost() const {return hCost;}
    double getFCost() const {return fCost;}

    void setGCost(double var) {gCost = var;}
    void setHCost(double var) {hCost = var;}
    void setFCost(double var) {fCost = var;}

    void updateCosts(const MapNode& dest) {
        /* diagonal element increase 1.4 ~= sqrt(2) */
        if (parent) {
            gCost = parent->gCost;
            gCost += ((abs(_x - parent->_x) + abs(_y - parent->_x)) > 1) ? 1.4 : 1;
        } else {
            /* origin */
            gCost = 0.0;
        }
        hCost = abs(_x - dest._x) + abs(_y - dest._y);
        fCost = gCost + hCost;
    }

    /* without consider propability */
    std::vector<MapNode> getAdjacents() {
        std::vector<MapNode> adjacents;

        //  horizontal
        adjacents.push_back(MapNode(_x + 1, _y, this));
        adjacents.push_back(MapNode(_x - 1, _y, this));
        adjacents.push_back(MapNode(_x, _y + 1, this));
        adjacents.push_back(MapNode(_x, _y - 1, this));

        //  diagonal
        adjacents.push_back(MapNode(_x + 1, _y + 1, this));
        adjacents.push_back(MapNode(_x + 1, _y - 1, this));
        adjacents.push_back(MapNode(_x - 1, _y + 1, this));
        adjacents.push_back(MapNode(_x - 1, _y - 1, this));

        return adjacents;
    }

    std::vector<MapNode> getPath() {
        std::vector<MapNode> path({*this});
        MapNode* curPos = this;
        while (curPos -> parent) {
            curPos = curPos -> parent;
            path.insert(path.begin(), *curPos);
        }

        return path;
    }

 private:
    double gCost;
    double hCost;
    double fCost;
    /* single parent */
    MapNode* parent;
};
// operator overloading
bool operator < (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost < rhs.fCost);
}
bool operator > (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost > rhs.fCost);
}
bool operator == (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost == rhs.fCost &&
            lhs.gCost == rhs.gCost &&
            lhs.hCost == rhs.hCost &&
            lhs.parent == rhs.parent);
}
bool operator <= (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost <= rhs.fCost);
}
bool operator >= (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost <= rhs.fCost);
}

std::ostream& operator << (std::ostream& out, const MapNode& n) {
    out << "(" << n.getX() << ", " << n.getY() << ") with fcost:"
        << n.getFCost();
    return out;
}

/**********************************************************************
 ** OpenList implemented in min Heap for fast query minimum f cost.
 **********************************************************************/

class OpenList {
 public:
    friend std::ostream& operator << (std::ostream& out, const OpenList& list);
    OpenList() {}
    explicit OpenList(std::vector<MapNode> nodes) {
        /* push into heap one after another */
        for (auto&& node : nodes) {
            heap.push_back(node);
            std::push_heap(heap.begin(), heap.end(), comparision);
        }
    }

    void pushNode(const MapNode& node) {
        heap.push_back(node);
        std::push_heap(heap.begin(), heap.end(), comparision);
    }

    void pushNodes(const std::vector<MapNode> nodes) {
        for (auto& node : nodes) {
            pushNode(node);
        }
    }

    MapNode& popNode() {
        std::pop_heap(heap.begin(), heap.end(), comparision);
        auto& minFCostNode = heap.back();
        heap.pop_back();

        return minFCostNode;
    }

    void modifyNode(const MapNode& oldNode, const MapNode& newNode) {
        for (auto&& n : heap) {
            if (n == oldNode) {
                n.fCost  = newNode.fCost;
                n.hCost  = newNode.hCost;
                n.gCost  = newNode.gCost;
                n.parent = newNode.parent;
            }
        }
        std::make_heap(heap.begin(), heap.end(), comparision);
    }

 private:
    std::greater<MapNode> comparision;
    std::vector<MapNode>  heap;
};

std::ostream& operator << (std::ostream& out, const OpenList& list) {
    for (auto&& node : list.heap) {
        out << node << std::endl;
    }
    return out;
}

/**********************************************************************
 ** Robot represent a robot information
 **********************************************************************/
class Robot {
 public:
    friend std::ostream& operator << (std::ostream& out, const Robot& robot);

    Robot() {
        name     = std::string("");
        position = Node();
    }
    explicit Robot(const Node& pos) {
        name     = std::string("anymous robot");
        position = pos;
    }
    Robot(std::string botName, const Node& pos) {
        name     = botName;
        position = pos;
    }

    void walk(const int xDiff, const int yDiff) {
        position.walk(xDiff, yDiff);
    }

    Node getPosition() const {return position;}
    std::string getName() const {return name;}



 private:
    std::string name;
    Node position;
};

std::ostream& operator << (std::ostream& out, const Robot& robot) {
    out << robot.name << " @ " << robot.position;
    return out;
}

/**********************************************************************
 **  Map containing information of space
 **********************************************************************/
class Map {
 public:
    explicit Map(std::vector<int8_t> rawMap) {
    }
    void updateMap(std::vector<int8_t> rawMap) {
    }

    std::vector<MapNode> getAvailableAdjacents(MapNode* const n, const MapNode& dest) {
        auto canidates = n->getAdjacents();
        /* TODO ... check possibility and if in Close List */
        for (auto itr = canidates.begin(); itr != canidates.end(); ) {
            if (itr->getX() < 0 || itr->getY() < 0) {
                itr = canidates.erase(itr);
            } else {
                itr->updateCosts(dest);
                ++itr;
            }
        }
        return canidates;
    }

    std::vector<MapNode> aStar(const MapNode& dest);

 private:
    std::vector<double> map;
    std::vector<bool>   closeList;
    OpenList            openList;         // heap based
    Robot               mobileBot;
};


#endif  // _A_STAR_
