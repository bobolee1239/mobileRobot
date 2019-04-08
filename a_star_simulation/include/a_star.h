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
#include "nav_msgs/OccupancyGrid.h"

#define MAP_OCCUPIED 100.0
/**********************************************************************
 ** Analog Node to represent a node in space
 **********************************************************************/
class Node {
 public:
    friend std::ostream& operator << (std::ostream& out, const Node& n);
    friend bool operator == (const Node& lhs, const Node& rhs);

    Node(const Node& node) : _x(node._x), _y(node._y) {}
    Node(double x, double y) : _x(x), _y(y) {}
    Node()  : _x(0), _y(0) {}

    double getX() const {return _x;}
    double getY() const {return _y;}
    Node& walk(const double xDiff, const double yDiff) {
        _x += xDiff;
        _y += yDiff;
        return *this;
    }
    Node& moveTo(const double x, const double y) {
        _x = x;
        _y = y;
        return *this;
    }

 protected:
    double _x;
    double _y;
};

std::ostream& operator << (std::ostream& out, const Node& n) {
    out << "(" << n.getX() << ", " << n.getY() << ")";
    return out;
}

bool operator == (const Node& lhs, const Node& rhs) {
    return (lhs._x == rhs._x) && (lhs._y == rhs._y);
}

/**********************************************************************
 ** - MapNode to represent a node in map
 ** - Digital Node to represent a node in space
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
        gCost += (((abs(_x - src->_x) + abs(_y - src->_y)) > 1) ? 1.4 : 1);
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

    Robot& walk(const double xDiff, const double yDiff) {
        position.walk(xDiff, yDiff);
        return *this;
    }

    Robot& moveTo(const double x, const double y) {
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

    explicit Map(const nav_msgs::OccupancyGrid& rawMap) {
        this->width      = rawMap.info.width;
        this->height     = rawMap.info.height;
        this->resolution = rawMap.info.resolution;
        this->originX    = rawMap.info.origin.position.x;
        this->originY    = rawMap.info.origin.position.y;

        /* build up map and close list */
        int8_t e;
        for (int i=0, j=rawMap.data.size(); i < j; ++i) {
            e = rawMap.data[i];
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

    explicit Map(std::vector<int8_t> rawMap, unsigned int width, unsigned int height) {
        this->width      = width;
        this->height     = height;
        this->resolution = 1.0;
        this->originX    = 0.0;
        this->originY    = 0.0;
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
    Map& updateMap(const nav_msgs::OccupancyGrid& rawMap) {
        int8_t e;
        for (int i=0, j=rawMap.data.size(); i < j; ++i) {
            e = rawMap.data[i];
            if (e < 0) {
                /* unknown node -> we cannot go there */
                mapNodes[i]->occupiedProb = e;
            } else {
                mapNodes[i]->occupiedProb = static_cast<double>(e) / MAP_OCCUPIED;
            }
        }
        reset();
        return *this;
    }
    Map& updateMap(const std::vector<int8_t> rawMap) {
        int8_t e;
        for (int i=0, j=rawMap.size(); i < j; ++i) {
            e = rawMap[i];
            if (e < 0) {
                /* unknown node -> we cannot go there */
                mapNodes[i]->occupiedProb = e;
            } else {
                mapNodes[i]->occupiedProb = static_cast<double>(e) / MAP_OCCUPIED;
            }
        }
        reset();
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
            if (e->occupiedProb >= 0) {
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
        /* check bounded and throw error */
        if (isOuttaMap(x, y)) {
            throw "Outta the bound of map!";
        }
        return mapNodes[x + y*width];
    }

    MapNode* at(const Node& n) {
        /* check bounded and throw error */
        if (isOuttaMap(n)) {
            throw "Outta the bound of map!";
        }
        return mapNodes[n.getX() + n.getY()*width];
    }

    Robot getRobot() const {return mobileBot;}

    Map& moveRobotTo(const double x, const double y) {
        Node dest = adc(Node(x, y));
        /* check bounded and throw error */
        if (isOuttaMap(dest)) {
            throw "[Move Failed] Outta the bound of map!";
        }
        mobileBot.moveTo(dest.getX(), dest.getY());
        return *this;
    }

    double isOccupiedAt(int x, int y) const {
        /* check bounded and throw error */
        if (isOuttaMap(x, y)) {
            throw "[Check Occupied Failed] Outta the bound of map!";
        }
        return mapNodes[x + y*width]->occupiedProb > 0.9;
    }

    double isOccupiedAt(const Node* const n) const {
        /* check bounded and throw error */
        if (isOuttaMap(n)) {
            throw "[Check Occupied Failed] Outta the bound of map!";
        }
        return mapNodes[n->getX() + (n->getY())*width]->occupiedProb > 0.9;
    }

    bool isOuttaMap(const Node* const n) const {
        /* TODO... Update the bounded */
        return ((n->getX() < 0) || (n->getX() > width - 1)
                    || (n->getY() < 0) || (n->getY() > height - 1));
    }

    bool isOuttaMap(const Node& n) const {
        /* TODO... Update the bounded */
        return ((n.getX() < 0) || (n.getX() > width - 1)
                    || (n.getY() < 0) || (n.getY() > height - 1));
    }

    bool isOuttaMap(int x, int y) const {
        /* TODO... Update the bounded */
        return ((x < 0) || (x > width - 1)
                    || (y < 0) || (y > height - 1));
    }

    bool inCloseList(const Node* const n) const {
        /* check bounded and throw error */
        if (isOuttaMap(n)) {
            throw "[Check In Close List Failed] Outta the bound of map!";
        }
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

    /**
     **  Digital to Analog Convetor
     **/
    std::vector<Node> dac(const std::vector<MapNode> digitalNodes) {
        std::vector<Node> realPath;
        for (auto&& dn : digitalNodes) {
            realPath.push_back(Node(originX + dn.getX() * resolution,
                                    originY + dn.getY() * resolution));
        }
        return realPath;
    }
    Node dac(const Node& digitalNode) {
        return Node(originX + digitalNode.getX() * resolution,
                    originY + digitalNode.getY() * resolution);
    }

    /**
     **  Analog to Digital Convetor
     **/
    Node adc(const Node& n) {
        /*
        std::cout << "resolution: " << this->resolution << std::endl
                  << "node x: " << n.getX() << std::endl
                  << "node y: " << n.getY() << std::endl
                  << "map origin x: " << originX << std::endl
                  << "map origin y: " << originY << std::endl;
        */
        return Node(static_cast<int>((n.getX() - originX) / resolution),
                    static_cast<int>((n.getY() - originY) / resolution));
    }

    Map& setOrigin(double x, double y) {
        this->originX = x;
        this->originY = y;
        return *this;
    }

    Map& setResolution(double res) {
        this->resolution = res;
        return *this;
    }

    /**************************************************************
     ** DESCRIPTION:
     **     1. Get MapNodes that arround the assigned node pointer
     **     2. Sifting nodes that not in map and in close list
     **     3. updateCost of the mapNode if it is available
     **************************************************************/
    std::vector<MapNode*> getAvailableAdjacents(const MapNode* const n) {
        /* find all adjacent nodes in the map */
        std::vector<MapNode*> adjacents;
/* DEBUG sake
        std::cout << "(" << n->_x + 1 << ", " << n->_y + 1 << ")\n"
                  << "(" << n->_x + 1 << ", " << n->_y - 1 << ")\n"
                  << "(" << n->_x - 1 << ", " << n->_y + 1 << ")\n"
                  << "(" << n->_x - 1 << ", " << n->_y - 1 << ")\n"
                  << "(" << n->_x + 1 << ", " << n->_y + 0 << ")\n"
                  << "(" << n->_x + 1 << ", " << n->_y + 0 << ")\n"
                  << "(" << n->_x + 0 << ", " << n->_y + 1 << ")\n"
                  << "(" << n->_x + 0 << ", " << n->_y + 1 << ")\n"
                  << std::endl;
*/

        //  horizontal
        if (!isOuttaMap(n->_x + 1, n->_y)) {
            adjacents.push_back(this->at(n->_x + 1, n->_y));
        }
        if (!isOuttaMap(n->_x - 1, n->_y)) {
            adjacents.push_back(this->at(n->_x - 1, n->_y));
        }
        if (!isOuttaMap(n->_x    , n->_y + 1)) {
            adjacents.push_back(this->at(n->_x    , n->_y + 1));
        }
        if (!isOuttaMap(n->_x    , n->_y - 1)) {
            adjacents.push_back(this->at(n->_x    , n->_y - 1));
        }

        //  diagonal
        if (!isOuttaMap(n->_x + 1, n->_y + 1)) {
            adjacents.push_back(this->at(n->_x + 1, n->_y + 1));
        }
        if (!isOuttaMap(n->_x + 1, n->_y - 1)) {
            adjacents.push_back(this->at(n->_x + 1, n->_y - 1));
        }
        if (!isOuttaMap(n->_x - 1, n->_y + 1)) {
            adjacents.push_back(this->at(n->_x - 1, n->_y + 1));
        }
        if (!isOuttaMap(n->_x - 1, n->_y - 1)) {
            adjacents.push_back(this->at(n->_x - 1, n->_y - 1));
        }

        for (auto itr = adjacents.begin(); itr != adjacents.end(); ) {
            /*********************************************
             **  1. Check it's not occupied by obstacle
             **  2. Check not in close list
             *********************************************/
            if (inCloseList(*itr) || isOccupiedAt(*itr)) {
                //  not available
                itr = adjacents.erase(itr);
            } else {
                //  available
                ++itr;
            }
        }
        return adjacents;
    }


    std::vector<Node> aStar(const Node& realDest) {
        /* clear closeList before calling A* algorithm */
        reset();

        /**
         ** [Digitalize the destination]
         ** find the closest node to the real destination
         **/
        Node dest = adc(realDest);

        /************************************************
         **  1. Add the staring position in open list.
         ************************************************/
        openList.pushNode(this->at(mobileBot.getPosition()));

        MapNode* walkTo;
#ifdef DEBUG
        std::cout << "========== A* BEGIN ========== " << std::endl;
#endif
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
                return dac(walkTo->getPath());
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
                                  << *tmpNodeInOpenList << std::endl;
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
        /* if path is empty => no path exists */
        return std::vector<Node>();
    }

 private:
    Robot                 mobileBot;      // vehichle in the map
    std::vector<bool>     closeList;      // same size of map, true: occuipied
    OpenList              openList;       // heap based
    std::vector<double>   occupiedProb;   // contains prob of the node
    std::vector<MapNode*> mapNodes;       // map nodes in the map
    unsigned int          width;          // # of sampling in width
    unsigned int          height;         // # of sampling in height
    double                resolution;     // samping space (Unit: meter)
    double                originX;        // origin of the map
    double                originY;        // origin of the map
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
