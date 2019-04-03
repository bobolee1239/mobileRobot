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
#include <iomanip>

#define MAP_OCCUPIED 100.0
/**********************************************************************
 ** Node to represent a node in space
 **********************************************************************/
class Node {
 public:
    friend std::ostream& operator << (std::ostream& out, const Node& n);
    friend bool operator == (const Node& lhs, const Node& rhs);

    Node(int x, int y) : _x(x), _y(y) {}
    Node(const Node& node) : _x(node._x), _y(node._y) {}
    Node()  : _x(0), _y(0) {}

    int getX() const {return _x;}
    int getY() const {return _y;}
    Node& walk(const int xDiff, const int yDiff) {
        _x += xDiff;
        _y += yDiff;
        return *this;
    }
    Node& moveTo(const int x, const int y) {
        _x = x;
        _y = y;
        return *this;
    }

 protected:
    int _x;
    int _y;
};

std::ostream& operator << (std::ostream& out, const Node& n) {
    out << "(" << n.getX() << ", " << n.getY() << ")";
    return out;
}

bool operator == (const Node& lhs, const Node& rhs) {
    return (lhs._x == rhs._x) && (lhs._y == rhs._y);
}

/**********************************************************************
 ** MapNode to represent a node in map
 **********************************************************************/

class MapNode : public Node {
 public:
    friend bool operator < (const MapNode& lhs, const MapNode& rhs);
    friend bool operator > (const MapNode& lhs, const MapNode& rhs);
    // friend bool operator == (const MapNode& lhs, const MapNode& rhs);
    friend bool operator <= (const MapNode& lhs, const MapNode& rhs);
    friend bool operator >= (const MapNode& lhs, const MapNode& rhs);
    friend std::ostream& operator << (std::ostream& out, const MapNode& n);
    friend class OpenList;
    friend class Map;

    MapNode() : Node(0, 0) {
         parent = NULL;
     }
    MapNode(int x, int y) : Node(x, y) {
        parent = NULL;
    }
    MapNode(int x, int y, MapNode* parentPtr) : Node(x, y) {
        parent = parentPtr;
    }
    explicit MapNode(const Node& n) : Node(n) {
        parent = NULL;
    }

    double getGCost() const {return gCost;}
    double getHCost() const {return hCost;}
    double getFCost() const {return fCost;}

    void setGCost(double var) {gCost = var;}
    void setHCost(double var) {hCost = var;}
    void setFCost(double var) {fCost = var;}

    MapNode& updateCosts(const MapNode& dest) {
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

        return *this;
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
        /*
#ifdef DEBUG
        std::cout << static_cast<Node>(*curPos) << " -> ";
#endif
        */
        while (curPos -> parent) {
            curPos = curPos -> parent;
        /*
#ifdef DEBUG
        std::cout << static_cast<Node>(*curPos) << " -> ";
#endif
*/
            path.insert(path.begin(), *curPos);
        }

        return path;
    }

 private:
    double gCost;
    double hCost;
    double fCost;
public:
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
/*
bool operator == (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost == rhs.fCost &&
            lhs.gCost == rhs.gCost &&
            lhs.hCost == rhs.hCost &&
            lhs.parent == rhs.parent);
}
*/
bool operator <= (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost <= rhs.fCost);
}
bool operator >= (const MapNode& lhs, const MapNode& rhs) {
    return (lhs.fCost <= rhs.fCost);
}

std::ostream& operator << (std::ostream& out, const MapNode& n) {
    out << "(" << n.getX() << ", " << n.getY()
        << ") with fcost:" << n.getFCost();
    if (n.parent) {
        out << " which inheriant from " << static_cast<Node>(*(n.parent));
    }
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

    bool empty() const {
        return heap.empty();
    }

    MapNode* contains(const MapNode& n) {
        auto itr = std::find(heap.begin(), heap.end(), n);
        if (heap.end() == itr) {
            return NULL;
        } else {
            return &(*itr);
        }
    }

    OpenList& pushNode(const MapNode& node) {
        heap.push_back(node);
        std::push_heap(heap.begin(), heap.end(), comparision);
        return *this;
    }

    OpenList& pushNodes(const std::vector<MapNode> nodes) {
        for (auto& node : nodes) {
            pushNode(node);
        }
        return *this;
    }

    MapNode popNode() {
        std::pop_heap(heap.begin(), heap.end(), comparision);
        auto minFCostNode = heap.back();
        heap.pop_back();

        return minFCostNode;
    }

    OpenList& modifyNode(const MapNode& oldNode, const MapNode& newNode) {
        for (auto&& n : heap) {
            if (n == oldNode) {
                n.fCost  = newNode.fCost;
                n.hCost  = newNode.hCost;
                n.gCost  = newNode.gCost;
                n.parent = newNode.parent;
            }
        }
        std::make_heap(heap.begin(), heap.end(), comparision);

        return *this;
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
        name     = std::string("anonymous bot");
        position = Node();
    }
    explicit Robot(const Node& pos) {
        name     = std::string("anonymous bot");
        position = pos;
    }
    Robot(std::string botName, const Node& pos) {
        name     = botName;
        position = pos;
    }

    Robot& walk(const int xDiff, const int yDiff) {
        position.walk(xDiff, yDiff);
        return *this;
    }

    Robot& moveTo(const int x, const int y) {
        position.moveTo(x, y);
        return *this;
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
 **  Map containing information of space and our robot
 **********************************************************************/
class Map {
 public:
    friend std::ostream& operator << (std::ostream& out, const Map& map);
    explicit Map(std::vector<int8_t> rawMap, unsigned int width, unsigned int height) {
        this->width  = width;
        this->height = height;
        /* build up map and close list */
        std::for_each(rawMap.begin(), rawMap.end(), [this](int8_t& e){
            if (e >= 0) {
                double prob = static_cast<double>(e) / MAP_OCCUPIED;
                map.push_back(prob);
                /***************************************************
                 ** Bot cannot move to somewhere with 0.5 prob that
                 ** was occupied.
                 ***************************************************/
                closeList.push_back(prob > 0.5);
            } else {
                /* unknown node -> we cannot go there */
                map.push_back(e);
                closeList.push_back(true);
            }
        });
    }

    /* In case we'll update the map */
    Map& updateMap(std::vector<int8_t> rawMap) {
        std::for_each(rawMap.begin(), rawMap.end(), [this](int8_t& e){
            if (e >= 0) {
                double prob = static_cast<double>(e) / MAP_OCCUPIED;
                map.push_back(prob);
                closeList.push_back(prob > 0.5);
            } else {
                map.push_back(e);
                closeList.push_back(true);
            }
        });
        return *this;
    }

    Map& resetCloseList() {
        closeList.clear();
        std::for_each(map.begin(), map.end(), [this](double& e){
            if (e >= 0) {
                closeList.push_back(e > 0.5);
            } else {
                closeList.push_back(true);
            }
        });
        return *this;
    }

    Robot getRobot() const {return mobileBot;}

    Map& moveRobotTo(const int x, const int y) {
        mobileBot.moveTo(x, y);
        return *this;
    }

    double isOccupiedAt(int x, int y) const {
        return map[x + y*width];
    }

    bool isOuttaMap(const Node& n) const {
        return ((n.getX() < 0) || (n.getX() > width)
                    || (n.getY() < 0) || (n.getY() > height));
    }

    bool inCloseList(const Node& n) const {
        /* transform 2D info th 1D info */
        return closeList[n.getX() + n.getY()*width];
    }

    bool isObstacle(const Node& n) const {
        return (map[n.getX() + n.getY()*width] > 0.9);
    }

    std::vector<bool> getCloseList() const { return closeList; }
    OpenList getOpenList() const { return openList; }
    std::vector<double> getMap() const { return map; }

    /**************************************************************
     ** DESCRIPTION:
     **     1. Get MapNodes that arround the assigned node pointer
     **     2. Sifting nodes that not in map and in close list
     **     3. updateCost of the mapNode if it is available
     **************************************************************/
    std::vector<MapNode> getAvailableAdjacents(MapNode* const n, const MapNode& dest) {
        auto canidates = n->getAdjacents();

        for (auto itr = canidates.begin(); itr != canidates.end(); ) {
            /**************************************
             **  1. Check it's inside the map
             **  2. Check not in close list
             **************************************/
            if (isOuttaMap(*itr) || inCloseList(*itr) || isObstacle(*itr)) {
                itr = canidates.erase(itr);
            } else {
                itr->updateCosts(dest);
                ++itr;
            }
        }
        return canidates;
    }


    std::vector<MapNode> aStar(const MapNode& dest) {
        /* clear closeList before calling A* algorithm */
        resetCloseList();
        /* if path is empty => no path exists */
        std::vector<MapNode> path;
        /************************************************
         **  1. Add the staring position in open list.
         ************************************************/
        openList.pushNode(MapNode(mobileBot.getPosition()));

        while (!openList.empty()) {
        /************************************************
         ** 2. Find the lowest F cost in open list and
         **    Add it to close list.
         **    [Done] if the node is the destination.
         ************************************************/
            auto walkTo = openList.popNode();
            std::cout << "walk to " << walkTo << std::endl;
            //  return the path if it's the destination
            if (walkTo == dest) {
                /* bottom up to get whole path */
#ifdef DEBUG
                std::cout << "A* done!" << std::endl;
#endif

                // return walkTo.getPath();
                return std::vector<MapNode>({walkTo});
            }
            //  add it to close list.
            closeList[walkTo.getX() + walkTo.getY() * width] = true;

         /***********************************************
          ** 3. Add adjacent nodes and
          **    Recalculate the F, G, H cost
          **
          ** 4. If new cost is better than before =>
          **    change the parent of the node
          ***********************************************/
            auto canidates = getAvailableAdjacents(&walkTo, dest);
#ifdef DEBUG
                    std::cout << "canidates(sifted): ";
                    for (auto&& c : canidates) {
                        std::cout << c << std::endl;
                    }
#endif
            MapNode* tmpNodeInOpenList;
            for (auto&& node : canidates) {
                if ((tmpNodeInOpenList = openList.contains(node))) {
                    if (*tmpNodeInOpenList > node) {
                        tmpNodeInOpenList -> parent = &walkTo;
                    }
                } else {
#ifdef DEBUG
                    std::cout << "Adding map node: " << node << "..." << std::endl;
#endif
                    openList.pushNode(node);
                }
            }
        }
        return path;
    }

 private:
    Robot               mobileBot;
    std::vector<bool>   closeList;      // same size of map, true: occuipied
    OpenList            openList;       // heap based
    std::vector<double> map;            // contains prob that node
                                        // might be occupied
    unsigned int        width;
    unsigned int        height;
};

std::ostream& operator << (std::ostream& out, const Map& map) {
    int botIdx = map.mobileBot.getPosition().getX()
                    + map.width * map.mobileBot.getPosition().getY();
    int idx;
    out << std::fixed << std::setprecision(2);
    out << "-----------------------------------------------------\n";
    for (int r = map.height - 1; r >= 0 ; --r) {
        for (int c = 0, w = map.width; c < w; ++c) {
            idx = c + r*w;
            if (botIdx == idx) {
                out << "****" << " | ";
            } else {
                out << map.map[idx] << " | ";
            }
        }
        out << std::endl;
    }
    out << "-----------------------------------------------------\n";
    return out;
}


#endif  // _A_STAR_
