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
    MapNode(int x, int y, double p) : Node(x, y), occupiedProb(p) {
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
    double getOccupiedProb() const {return occupiedProb;}

    MapNode& setGCost(double var) {
        gCost = var;
        return *this;
    }
    MapNode& setHCost(double var) {
        hCost = var;
        return *this;
    }
    MapNode& setFCost(double var) {
        fCost = var;
        return *this;
    }
    MapNode& setOccupiedProb(double p) {
        occupiedProb = p;
        return *this;
    }

    /**
     **  computeCosts:
     **    @para src  : the adjacent node we come from
     **    @para dest : the destination we're about to go
     **/
    MapNode& computeCostsNSetParent(MapNode* src, const Node& dest) {
        /* compute g cost */
        gCost  = src->gCost;
        gCost += ((abs(_x - src->_x) + abs(_y - src->_y)) > 1) ? 1.4 : 1;
        /* compute h cost */
        hCost  = abs(_x - dest.getX()) + abs(_y - dest.getY());
        /* compute f cost */
        fCost  = gCost + hCost;
        /* set parent */
        parent = src;
        return *this;
    }
    bool reComputeCostsNChangeParent(MapNode* src, const Node& dest) {
        double tmpGCost = 0.0;
        double tmpHCost = 0.0;
        double tmpFCost = 0.0;
        /* compute g cost */
        tmpGCost  = src->gCost;
        tmpGCost += ((abs(_x - src->_x) + abs(_y - src->_y)) > 1) ? 1.4 : 1;
        /* compute h cost */
        tmpHCost  = abs(_x - dest.getX()) + abs(_y - dest.getY());
        /* compute f cost and change parent if neccessary */
        if ((tmpFCost  =  tmpGCost + tmpHCost) < fCost) {
            parent = src;
            fCost = tmpFCost;
            gCost = tmpGCost;
            hCost = tmpHCost;
            //  return we modify the cost and parent.
            return true;
        } else {
            //  return that we do nothing.
            return false;
        }
    }

    /* without consider propability */
    /*
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
    */

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
    double occupiedProb;
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
    OpenList() {
        /* greater than comparision for minimum heap */
        comparison = [](const MapNode* lhs, const MapNode* rhs) {
            return lhs->fCost > rhs->fCost;
        };
    }

    bool empty() const {
        return heap.empty();
    }

    OpenList& clear() {
        heap.clear();
        return *this;
    }

    /**
     **     contains
     **       @para mapNode : node to match
     **
     **       Returns:
     **         - the actual node in the open list
     **/
    MapNode* contains(const MapNode* const n) {
        for (auto&& nPtr : heap) {
            if (*nPtr == *n) {
                return nPtr;
            }
        }
        return NULL;
    }

    OpenList& pushNode(MapNode* const nodePtr) {
        heap.push_back(nodePtr);
        std::push_heap(heap.begin(), heap.end(), comparison);
        return *this;
    }

    MapNode* popNode() {
        std::pop_heap(heap.begin(), heap.end(), comparison);
        auto minFCostNode = heap.back();
        heap.pop_back();

        return minFCostNode;
    }

    OpenList& reHeap() {
        std::make_heap(heap.begin(), heap.end(), comparison);
        return *this;
    }

 private:
    bool (*comparison)(const MapNode*, const MapNode*);
    std::vector<MapNode*>  heap;
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
        int8_t e;
        for (int i=0, j=rawMap.size(); i < j; ++i) {
            e = rawMap[i];
            if (e < 0) {
                /* unknown node -> we cannot go there */
                mapNodes.push_back(new MapNode(i%width, i/width, e));
                closeList.push_back(true);
            } else {
                double prob = static_cast<double>(e) / MAP_OCCUPIED;
                mapNodes.push_back(new MapNode(i%width, i/width, prob));
                /***************************************************
                 ** Bot cannot move to somewhere with 0.5 prob that
                 ** was occupied.
                 ***************************************************/
                closeList.push_back(prob > 0.5);
            }
        }
    }

    ~Map() {
        for (auto&& nPtr : mapNodes) {
            delete nPtr;
        }
    }

    /* In case we'll update the map */
    Map& updateMap(std::vector<int8_t> rawMap) {
        /* clear whole map and rebuild */
        this->~Map();
        int8_t e;
        for (int i=0, j=rawMap.size(); i < j; ++i) {
            e = rawMap[i];
            if (e < 0) {
                /* unknown node -> we cannot go there */
                mapNodes.push_back(new MapNode(i%width, i/width, e));
                closeList.push_back(true);
            } else {
                double prob = static_cast<double>(e) / MAP_OCCUPIED;
                mapNodes.push_back(new MapNode(i%width, i/width, prob));
                /***************************************************
                 ** Bot cannot move to somewhere with 0.5 prob that
                 ** was occupied.
                 ***************************************************/
                closeList.push_back(prob > 0.5);
            }
        }
        return *this;
    }

    Map& reset() {
        /**
         ** 1. Reset close list
         ** 2. Reset parent of all map nodes
         ** 3. Reset empty open list
         **/
        closeList.clear();
        std::for_each(mapNodes.begin(), mapNodes.end(), [this](MapNode* e){
            if (e >= 0) {
                closeList.push_back(e->occupiedProb > 0.5);
            } else {
                closeList.push_back(true);
            }
            e->parent = NULL;
        });
        openList.clear();

        return *this;
    }

    MapNode* at(int x, int y) const {
        return mapNodes[x + y*width];
    }

    MapNode* at(const Node& n) {
        return mapNodes[n.getX() + n.getY()*width];
    }

    Robot getRobot() const {return mobileBot;}

    Map& moveRobotTo(const int x, const int y) {
        mobileBot.moveTo(x, y);
        return *this;
    }

    double isOccupiedAt(int x, int y) const {
        return mapNodes[x + y*width]->occupiedProb > 0.9;
    }

    double isOccupiedAt(const Node* const n) const {
        return mapNodes[n->getX() + (n->getY())*width]->occupiedProb > 0.9;
    }

    bool isOuttaMap(const Node* const n) const {
        return ((n->getX() < 0) || (n->getX() > width)
                    || (n->getY() < 0) || (n->getY() > height));
    }

    bool isOuttaMap(int x, int y) const {
        return ((x < 0) || (x > width)
                    || (y < 0) || (y > height));
    }

    bool inCloseList(const Node* const n) const {
        /* transform 2D info th 1D info */
        return closeList[n->getX() + n->getY()*width];
    }

    /*
    bool isObstacle(const Node& n) const {
        return (map[n.getX() + n.getY()*width] > 0.9);
    }
    */

    std::vector<bool> getCloseList() const { return closeList; }
    OpenList getOpenList() const { return openList; }
    std::vector<MapNode*> getMap() const { return mapNodes; }

    /**************************************************************
     ** DESCRIPTION:
     **     1. Get MapNodes that arround the assigned node pointer
     **     2. Sifting nodes that not in map and in close list
     **     3. updateCost of the mapNode if it is available
     **************************************************************/
    std::vector<MapNode*> getAvailableAdjacents(const MapNode* const n) {
        /* find all adjacent nodes */
        std::vector<MapNode*> adjacents;
        //  horizontal
        adjacents.push_back(this->at(n->_x + 1, n->_y));
        adjacents.push_back(this->at(n->_x - 1, n->_y));
        adjacents.push_back(this->at(n->_x    , n->_y + 1));
        adjacents.push_back(this->at(n->_x    , n->_y - 1));

        //  diagonal
        adjacents.push_back(this->at(n->_x + 1, n->_y + 1));
        adjacents.push_back(this->at(n->_x + 1, n->_y - 1));
        adjacents.push_back(this->at(n->_x - 1, n->_y + 1));
        adjacents.push_back(this->at(n->_x - 1, n->_y - 1));

        for (auto itr = adjacents.begin(); itr != adjacents.end(); ) {
            /**************************************
             **  1. Check it's inside the map
             **  2. Check not in close list
             **************************************/
            if (isOuttaMap(*itr) || inCloseList(*itr) || isOccupiedAt(*itr)) {
                //  not available
                itr = adjacents.erase(itr);
            } else {
                //  available
                ++itr;
            }
        }
        return adjacents;
    }


    std::vector<MapNode> aStar(const Node& dest) {
#ifdef DEBUG
        std::cout << "========== A* BEGIN ========== " << std::endl;
#endif
        /* clear closeList before calling A* algorithm */
        reset();
        /* if path is empty => no path exists */
        std::vector<MapNode> path;
        /************************************************
         **  1. Add the staring position in open list.
         ************************************************/
        openList.pushNode(this->at(mobileBot.getPosition()));

        MapNode* walkTo;
        while (!openList.empty()) {
        /************************************************
         ** 2. Find the lowest F cost in open list and
         **    Add it to close list.
         **    [Done] if the node is the destination.
         ************************************************/
            walkTo = openList.popNode();
#ifdef DEBUG
            std::cout << " - walk to " << *walkTo << std::endl;
#endif
            /* return the path if it's the destination */
            if (*walkTo == dest) {
                /* bottom up to get whole path */
#ifdef DEBUG
                std::cout << "========== A* END ========== " << std::endl;
#endif
                return walkTo->getPath();
            }
            /*  add walked node to close list. */
            closeList[walkTo->getX() + walkTo->getY() * width] = true;

         /***********************************************
          ** 3. Add adjacent nodes and
          **    Recalculate the F, G, H cost
          **
          ** 4. If new cost is better than before =>
          **    change the parent of the node
          ***********************************************/
            auto adjacents = getAvailableAdjacents(walkTo);
            MapNode* tmpNodeInOpenList;
            for (auto&& nodePtr : adjacents) {
                if ((tmpNodeInOpenList = openList.contains(nodePtr))) {
                    /**
                     **     If it's already in open list
                     **  => 1. Recompute Costs
                     **  => 2.|(1) Change Parent if neccessary
                     **       |(2) Do nothing
                     **/
                    if (tmpNodeInOpenList->reComputeCostsNChangeParent(walkTo, dest)) {
#ifdef DEBUG
                        std::cout << "\t[MODIFIED] "
                                  << tmpNodeInOpenList << std::endl;
#endif
                        openList.reHeap();
                    }
                } else {
                    /**
                     **     If it's not in open list
                     **  => 1. Compute Costs
                     **  => 2. Add it into open list
                     **/
                    nodePtr->computeCostsNSetParent(walkTo, dest);
                    openList.pushNode(nodePtr);
#ifdef DEBUG
                    std::cout << "\t * Adding " << *nodePtr << std::endl;
#endif
                }
            }
#ifdef DEBUG
                    std::cout << std::endl;
#endif
        }
        return path;
    }

 private:
    Robot                 mobileBot;
    std::vector<bool>     closeList;      // same size of map, true: occuipied
    OpenList              openList;       // heap based
    std::vector<double>   occupiedProb;   // contains prob of the node
    std::vector<MapNode*> mapNodes;       // map nodes in the map
    unsigned int          width;
    unsigned int          height;
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
                out << map.at(c, r)->getOccupiedProb() << " | ";
            }
        }
        out << std::endl;
    }
    out << "-----------------------------------------------------\n";
    return out;
}


#endif  // _A_STAR_
