#include <bits/stdc++.h>
using namespace std;

static const double INF = 1e15;
static const int INF_INT = 1e9+7;
double hyperperiod_end = 400;
static const int KMAX = 3;

typedef pair<int, double> Edge;

auto makeKey (int a, int b) {
    return ((long long)a << 32) | (b & 0xffffffffLL);
}


//-======================= Structs required for Computation Model =============================================================================
struct Task{
    int id;
    int computation_units;
    vector<int> successors;
    vector<int> predecessors;

    Task() : id(-1), computation_units(0) {}

    Task(int id_, int computation_units_)
        : id(id_), computation_units(computation_units_){}
};

struct Message {
    int id;
    int srcTaskId;
    int dstTaskId;
    int size;

    Message(int id_, int src, int dst, int size_)
        : id(id_), srcTaskId(src), dstTaskId(dst), size(size_) {}
};

//==========================================================================================================================================

struct Cost {
    double a;  // multiplied by msgSize
    double b;  // fixed overhead
};

Cost addCost(const Cost &c1, const Cost &c2) {
    return { c1.a + c2.a, c1.b + c2.b };
}

double costVal(const Cost &c, int msgSize) {
    return c.a * msgSize + c.b;
}

//======================Structs required for Platform Model ---------------------------------------------------------------------------------
struct Processor {
    int id;
    double speed;
    double startup_delay;

    Processor() : id(-1), speed(0), startup_delay(0) {}

    Processor(int id_, double speed_, double startup_delay_)
        : id(id_), speed(speed_), startup_delay(startup_delay_) {}

};

struct BusyInterval {
    double start;
    double end;
    BusyInterval(double s, double e) : start(s), end(e) {}
};

struct BusyIntervalComparator {
    bool operator()(const BusyInterval &a, const BusyInterval &b) const {
        return a.start < b.start;
    }
};

unordered_map<int, multiset<BusyInterval, BusyIntervalComparator>> processorBusyMap;


struct Link {
    int id;
    int endpoint1;
    int endpoint2;
    double bandwidth;

    Link() : id(-1), endpoint1(-1), endpoint2(-1), bandwidth(0.0) {}

    Link(int id_, int ep1, int ep2, double bandwidth_)
        : id(id_), endpoint1(ep1), endpoint2(ep2), bandwidth(bandwidth_) {}
};

struct GateControlEntry {
    double departure_start;
    double departure_end;

    GateControlEntry(double ds, double de)
        : departure_start(ds), departure_end(de) {}
};

struct GateControlEntryComparator {
    bool operator()(const GateControlEntry &a, const GateControlEntry &b) const {
        return a.departure_start < b.departure_start;
    }
};

struct LinkSchedule {
    multiset<GateControlEntry, GateControlEntryComparator> busyIntervals;

    double reserveSlot(double arrival_time, double required_duration) {
        double free_start = arrival_time;

        for (auto &entry : busyIntervals) {
            if (free_start + required_duration <= entry.departure_start) {
                busyIntervals.insert({free_start, free_start + required_duration});
                return free_start;
            }

            if (free_start >= entry.departure_start && free_start < entry.departure_end) {
                free_start = entry.departure_end;
            }

            if (free_start + required_duration > hyperperiod_end) {
                return -1;
            }
        }

        if (free_start + required_duration > hyperperiod_end) {
            return -1;
        }
        busyIntervals.insert({free_start, free_start + required_duration});
        return free_start;
    }
};

unordered_map<int, LinkSchedule> linkSchedules;

struct Switch {
    int id;
    double startup_delay;

    vector<int> connectedLinks;

    Switch() : id(-1), startup_delay(0) {}

    Switch(int id_, double startup_delay_)
        : id(id_), startup_delay(startup_delay_) {}

    double reserveSlotOnLink(int link_id, double arrival_time,
                             double required_duration) {

        return linkSchedules[link_id].reserveSlot(arrival_time, required_duration);
    }
};

//==========================================================================================================================================



vector<Link> links;
vector<Task> tasks;

unordered_map<int,Task> taskById;
unordered_map<int,Processor> processorById;
unordered_map<int,Switch> switchById;

map<pair<int,int>,int> task_pair_to_msg;

vector<Processor> processors;
vector<Switch> switches;
int nodeCount;
vector<double> nodeStartupDelay;

unordered_map<long long,int> linkMap; 
unordered_map<int, Link> linkById;

vector<vector<double>> execution_times;
vector<vector<Cost>> dist;
vector<vector<vector<int>>> shortest_path;

vector<vector<double>> rank2D;
vector<double> ranks;
vector<double> priority;

unordered_map<int,double> finishTimeOfTask;
unordered_map<int,int> assignedProcOfTask;

//-================ Taking care of the preloading=====================================================

double reserveSlotForProcessor(int procId, double arrivalTime, double taskDuration, int dummy) {
    auto &busySet = processorBusyMap[procId];
    double freeStart = arrivalTime;

    for (auto &interval : busySet) {
        if (freeStart >= interval.start && freeStart < interval.end) {
            freeStart = interval.end;
        }
    }
    
    bool validSlot = true;
    double taskEnd = freeStart + taskDuration;
    
    if (taskEnd > hyperperiod_end) {
        return -1.0;
    }
    
    for (auto &interval : busySet) {
        if (!(taskEnd <= interval.start || freeStart >= interval.end)) {
            validSlot = false;
            freeStart = interval.end;
            break;
        }
    }
    
    if (validSlot) {
        if (dummy) busySet.insert({freeStart, taskEnd});
        return freeStart;
    }
    
    return reserveSlotForProcessor(procId, freeStart, taskDuration, dummy);
}



struct AStarResult {
    double finishTime;
    unordered_map<int, LinkSchedule> linkSchedules;
    vector<int> bestPath;
};

// A structure to hold a state in the A* search.
struct AStarState {
    int node;                       // current node id
    double time;                    // current time (including waiting/reservations)
    double f;                       // f = g + h (current time + heuristic estimate)
    vector<int> path;               // path taken so far
    unordered_map<int, LinkSchedule> linkSchedules;  // ephemeral link schedules along this branch
};

vector<vector<pair<int,int>>> buildGraphForAStar(int nodeCount, const vector<Link> &links) {
    vector<vector<pair<int,int>>> graph(nodeCount + 1);
    for (auto &lk : links) {
        // Assuming bidirectional links.
        graph[lk.endpoint1].push_back({lk.endpoint2, lk.id});
        graph[lk.endpoint2].push_back({lk.endpoint1, lk.id});
    }
    return graph;
}

//-=================== Rank Calculation =================================================================================================================
double getRank(Task t, Processor p) {
    if (rank2D[t.id][p.id] >= 0.0) {
        return rank2D[t.id][p.id];
    }

    if (execution_times[t.id][p.id] >= INF_INT) {
        rank2D[t.id][p.id] = -INF;
        return rank2D[t.id][p.id];
    }

    double base = execution_times[t.id][p.id];

    if (t.successors.empty()) {
        rank2D[t.id][p.id] = base;
        return base;
    }

    double bestSuccessorTerm = -INF;

    for(auto succ: t.successors){
        double minVal = INF;

        for(auto pw: processors){
            if(execution_times[succ][pw.id] < INF_INT){
                double candidate = getRank(taskById[succ],pw) + execution_times[succ][pw.id] + costVal(dist[p.id][pw.id],task_pair_to_msg[{t.id,succ}]);
                if(candidate < minVal){
                    minVal = candidate;
                }
            }
        }
        if (minVal > bestSuccessorTerm) {
            bestSuccessorTerm = minVal;
        }
    }

    rank2D[t.id][p.id] = base + bestSuccessorTerm;
    return rank2D[t.id][p.id];
}


void computeAllRanks() {
    rank2D.assign(tasks.size() + 1, vector<double>(processors.size() + 1, -1.0));
    ranks.assign(tasks.size() + 1, -1.0);

    for (auto t: tasks) {
        for (auto p: processors) {
            if (execution_times[t.id][p.id] < INF_INT) {
                getRank(t, p);
            }
        }
    }

    for (auto t: tasks) {
        double best = -INF;
        for (auto p:processors) {
            if (rank2D[t.id][p.id] > best) {
                best = rank2D[t.id][p.id];
            }
        }
        ranks[t.id] = best;
    }
}
//==================================================================================================================================


//-======================= Priority Calculation =====================================================================================
double getPriority(Task t){
    double sum = 0, priority=0;
    int cnt=0;
    for(auto p: processors){
        if(rank2D[t.id][p.id]>=0){
            cnt++;
            sum += rank2D[t.id][p.id];
        }
    }
    priority = sum*t.successors.size()/(1.0*cnt);
    return priority;
}

void computeAllPriority(){
    priority.assign(tasks.size()+1,-1);

    for(auto t: tasks){
        priority[t.id] = getPriority(t);
    }

    for(int i=0; i<tasks.size();i++){
        for(auto t: tasks){
            for(auto it: t.successors){
                if(priority[it] >= priority[t.id]){
                    priority[t.id] = priority[it] + 0.01;
                }
            }
        }
    }
}
//============================================================

vector<int> createTaskList() {
    vector<int> taskList;
    for (auto &t : tasks) {
        taskList.push_back(t.id);
    }
    sort(taskList.begin(), taskList.end(), [&](int a, int b){
        return priority[a] > priority[b];
    });
    return taskList;
}

vector<int> create_message_priority_list(Task currentTask){
    if (currentTask.predecessors.empty()) {
        return {};
    }

    vector<double> RTs;
    vector<int> datas;
    vector<int> predTasks = currentTask.predecessors;

    for (int predId : predTasks) {
        double rt = finishTimeOfTask.at(predId);
        int d     = task_pair_to_msg.at({predId, currentTask.id});
        RTs.push_back(rt);
        datas.push_back(d);
    }

    double minRT = *min_element(RTs.begin(), RTs.end());
    double maxRT = *max_element(RTs.begin(), RTs.end());
    int minD = *min_element(datas.begin(), datas.end());
    int maxD = *max_element(datas.begin(), datas.end());


    vector<pair<int,double>> msgWithPriority;
    for (size_t i = 0; i < predTasks.size(); i++) {
        double RRT_j = 0.0, RD_j = 0.0;
    
        if (maxRT > minRT) {
            RRT_j = (maxRT - RTs[i]) / (maxRT - minRT);
        } else if (predTasks.size() > 1) {
            RRT_j = 0.5;
        }
        if (maxD > minD) {
            RD_j = (datas[i] - minD) / double(maxD - minD);
        } else if (predTasks.size() > 1) {
            RD_j = 0.5;
        }
        
        double msg_priority = 0.5 * RRT_j + 0.5 * RD_j;
        msgWithPriority.push_back({ predTasks[i], msg_priority });
    }
        
    sort(msgWithPriority.begin(), msgWithPriority.end(),
    [](auto &a, auto &b) {
        return a.second > b.second; 
    });

    vector<int> sortedPreds;
    for (auto mp : msgWithPriority) {
        sortedPreds.push_back(mp.first);
    }
    return sortedPreds;
}


double ephemeral_reserveLink(LinkSchedule &ls, double arrivalTime, double duration){
    double freeStart = arrivalTime;

    for (auto &entry : ls.busyIntervals){
        if (freeStart + duration <= entry.departure_start){
            ls.busyIntervals.insert({freeStart, freeStart + duration});
            return freeStart;
        }
        if (freeStart >= entry.departure_start && freeStart < entry.departure_end){
            freeStart = entry.departure_end;
        }
        if (freeStart + duration > hyperperiod_end){
            return -1.0;
        }
    }

    if (freeStart + duration > hyperperiod_end){
        return -1.0;
    }
    ls.busyIntervals.insert({freeStart, freeStart + duration});
    return freeStart;
}


AStarResult schedule_message_astar_beam(
    int srcNode, int dstNode,
    double msgStart, int msgSize,
    const unordered_map<int, LinkSchedule>& initLinkSchedules,
    const vector<vector<pair<int,int>>> &graph,
    int beamWidth
) {
    auto cmp = [](const AStarState &a, const AStarState &b) {
        return a.f > b.f;
    };
    priority_queue<AStarState, vector<AStarState>, decltype(cmp)> open(cmp);
    
    auto heuristic = [&](int node) -> double {
        return costVal(dist[node][dstNode], msgSize);
    };
    
    AStarState startState;
    startState.node = srcNode;
    startState.time = msgStart;
    startState.f = msgStart + heuristic(srcNode);
    startState.path.push_back(srcNode);
    startState.linkSchedules = initLinkSchedules;
    open.push(startState);
    
    while (!open.empty()) {

        vector<AStarState> currentLevel;
        int currentPathSize = open.top().path.size();
        while (!open.empty() && open.top().path.size() == currentPathSize) {
            currentLevel.push_back(open.top());
            open.pop();
        }

        sort(currentLevel.begin(), currentLevel.end(), [](const AStarState &a, const AStarState &b) {
            return a.f < b.f;
        });
        if (currentLevel.size() > (size_t)beamWidth) {
            currentLevel.resize(beamWidth);
        }

        vector<AStarState> nextLevel;
        for (auto &state : currentLevel) {

            if (state.node == dstNode) {
                return AStarResult{ state.time, state.linkSchedules, state.path };
            }

            for (auto &nbrPair : graph[state.node]) {
                int nbr = nbrPair.first;
                int linkId = nbrPair.second;

                auto newLinkSchedules = state.linkSchedules;

                double bandwidth = linkById.at(linkId).bandwidth;
                double duration = (msgSize / bandwidth) + nodeStartupDelay[state.node];

                double reservedStart = ephemeral_reserveLink(newLinkSchedules[linkId], state.time, duration);
                if (reservedStart < 0.0) {

                    continue;
                }
                double newTime = reservedStart + duration;
                if (newTime > hyperperiod_end) continue;
                AStarState newState;
                newState.node = nbr;
                newState.time = newTime;
                newState.path = state.path;
                newState.path.push_back(nbr);
                newState.linkSchedules = newLinkSchedules;
                newState.f = newTime + heuristic(nbr);
                nextLevel.push_back(newState);
            }
        }

        if (nextLevel.empty()) {
            return AStarResult{ -1.0, initLinkSchedules, {} };
        }

        sort(nextLevel.begin(), nextLevel.end(), [](const AStarState &a, const AStarState &b) {
            return a.f < b.f;
        });
        while (nextLevel.size() > (size_t)beamWidth) {
            nextLevel.pop_back();
        }

        for (auto &s : nextLevel) {
            open.push(s);
        }
    }
    return AStarResult{ -1.0, initLinkSchedules, {} };
}

double schedule_messages_astar(
    int currentTaskId,
    int candidateProcId,
    const vector<int> &sortedPredecessors,
    unordered_map<int, LinkSchedule> &revertLinkSchedules
) {
    double totalCommFinish = 0.0;
    vector<vector<pair<int,int>>> astarGraph = buildGraphForAStar(nodeCount, links);

    for (int predId : sortedPredecessors) {
        int predProcId = assignedProcOfTask.at(predId);
        double msgStart = finishTimeOfTask.at(predId);
        int msgSize = task_pair_to_msg.at({predId, currentTaskId});
        
        AStarResult result = schedule_message_astar_beam(predProcId, candidateProcId, msgStart, msgSize, revertLinkSchedules, astarGraph, 3);
        
        if (result.finishTime < 0) {
            return -1.0;
        }
        
        revertLinkSchedules = result.linkSchedules;

        totalCommFinish = max(totalCommFinish, result.finishTime);
    }
    
    return totalCommFinish;
}


int main()
{
    ios::sync_with_stdio(0);
    cin.tie(0);

    auto start = chrono::high_resolution_clock::now();

//===========Creation of Computation Model===========================================
    Task t1(1,60);
    Task t2(2,80);
    Task t3(3,56);
    Task t4(4,70);
    Task t5(5,50);


    t1.successors.push_back(t2.id);
    t1.successors.push_back(t3.id);
    t1.successors.push_back(t4.id);
    t2.successors.push_back(t5.id);
    t3.successors.push_back(t5.id);
    t4.successors.push_back(t5.id);
    
    t2.predecessors.push_back(t1.id);
    t3.predecessors.push_back(t1.id);
    t4.predecessors.push_back(t1.id);

    t5.predecessors.push_back(t2.id);
    t5.predecessors.push_back(t3.id);
    t5.predecessors.push_back(t4.id);


    Message m1(1,t1.id,t2.id,32);
    Message m2(2,t1.id,t3.id,40);
    Message m3(3,t1.id,t4.id,48);
    Message m4(4,t2.id,t5.id,56);
    Message m5(5,t3.id,t5.id,64);
    Message m6(6,t4.id,t5.id,48);
//===================================================================================

//========Creation of Platform Model=================================================
    Processor p1(1,6,0);
    Processor p2(2,8,1);
    Processor p3(3,7,1);
    Processor p4(4,5,2);

    Switch s1(5,0);
    Switch s2(6,1);
    Switch s3(7,2);
    Switch s4(8,1);

    Link l1(1,p1.id,s1.id,5);
    Link l2(2,s1.id,s2.id,7);
    Link l3(3,s2.id,p2.id,6);
    Link l4(4,s1.id,s3.id,6);
    Link l5(5,s2.id,s4.id,5);
    Link l6(6,s2.id,p4.id,8);
    Link l7(7,s3.id,p3.id,4);
    Link l8(8,s3.id,s4.id,7);
//=====================================================================================


    task_pair_to_msg[{t1.id,t2.id}] = m1.size;
    task_pair_to_msg[{t1.id,t3.id}] = m2.size;
    task_pair_to_msg[{t1.id,t4.id}] = m3.size;
    task_pair_to_msg[{t2.id,t5.id}] = m4.size;
    task_pair_to_msg[{t3.id,t5.id}] = m5.size;
    task_pair_to_msg[{t4.id,t5.id}] = m6.size;


//-=========Building Adjacency lists for the graphs ===================================
    links = {l1,l2,l3,l4,l5,l6,l7,l8};
    tasks = {t1,t2,t3,t4,t5};
    processors = {p1,p2,p3,p4};
    switches = {s1,s2,s3,s4};
    nodeCount = processors.size() + switches.size();
    execution_times.assign(tasks.size()+1, vector<double>(processors.size()+1));
    dist.assign(nodeCount + 1, vector<Cost>(nodeCount + 1, Cost{0.0, INF}));

    for(auto it: tasks){
        taskById[it.id]=it;
    }
    for(auto it: processors){
        processorById[it.id]=it;
    }
    for(auto it: switches){
        switchById[it.id]=it;
    }



    execution_times[t1.id][p1.id] = 15;
    execution_times[t1.id][p2.id] = INF;
    execution_times[t1.id][p3.id] = INF;
    execution_times[t1.id][p4.id] = INF;

    execution_times[t2.id][p1.id] = INF;
    execution_times[t2.id][p2.id] = 20;
    execution_times[t2.id][p3.id] = 25;
    execution_times[t2.id][p4.id] = INF;

    execution_times[t3.id][p1.id] = INF;
    execution_times[t3.id][p2.id] = 10;
    execution_times[t3.id][p3.id] = 20;
    execution_times[t3.id][p4.id] = INF;

    execution_times[t4.id][p1.id] = INF;
    execution_times[t4.id][p2.id] = 30;
    execution_times[t4.id][p3.id] = 15;
    execution_times[t4.id][p4.id] = INF;

    execution_times[t5.id][p1.id] = INF;
    execution_times[t5.id][p2.id] = INF;
    execution_times[t5.id][p3.id] = INF;
    execution_times[t5.id][p4.id] = 30;


    double messageSizeInBits = 1.0;
    // vector<vector<pair<int,int>>> adjacency = buildGraphForAStar(nodeCount, links);

//======================================================================================


    vector<vector<int>> nextHop(nodeCount + 1, vector<int>(nodeCount + 1, -1));
    dist.assign(nodeCount + 1, vector<Cost>(nodeCount + 1, Cost{0.0, INF}));


    auto makeKey = [&](int a, int b) {
        return ((long long)a << 32) | (b & 0xffffffffLL);
    };

    nodeStartupDelay.assign(nodeCount + 1, 0.0);
    nodeStartupDelay[p1.id] = p1.startup_delay;
    nodeStartupDelay[p2.id] = p2.startup_delay;
    nodeStartupDelay[p3.id] = p3.startup_delay;
    nodeStartupDelay[p4.id] = p4.startup_delay;
    nodeStartupDelay[s1.id] = s1.startup_delay;  
    nodeStartupDelay[s2.id] = s2.startup_delay;
    nodeStartupDelay[s3.id] = s3.startup_delay;
    nodeStartupDelay[s4.id] = s4.startup_delay;

    for (int i = 1; i <= nodeCount; i++) {
        dist[i][i] = Cost{0.0, 0.0};
        nextHop[i][i] = i;
    }

    for (auto &lk : links) {
        linkById[lk.id] = lk;

        int u = lk.endpoint1, v = lk.endpoint2;
        
        double aLink = 1.0 / lk.bandwidth;

        Cost costUV = { aLink, nodeStartupDelay[u] };

        Cost costVU = { aLink, nodeStartupDelay[v] };

        double testMsg = 1.0;
        
        if (costVal(costUV, testMsg) < costVal(dist[u][v], testMsg)) {
            dist[u][v] = costUV;
            nextHop[u][v] = v;
            linkMap[makeKey(u, v)] = lk.id;
        }

        if (costVal(costVU, testMsg) < costVal(dist[v][u], testMsg)) {
            dist[v][u] = costVU;
            nextHop[v][u] = u;
            linkMap[makeKey(v, u)] = lk.id;
        }
    }

    double testMsg = 1.0;
    for (int k = 1; k <= nodeCount; k++) {
        for (int i = 1; i <= nodeCount; i++) {
            for (int j = 1; j <= nodeCount; j++) {
                Cost newCost = addCost(dist[i][k], dist[k][j]);
                if (costVal(newCost, testMsg) < costVal(dist[i][j], testMsg)) {
                    dist[i][j] = newCost;
                    nextHop[i][j] = nextHop[i][k];
                }
            }
        }
    }

    shortest_path.assign(nodeCount + 1, vector<vector<int>>(nodeCount + 1));

    function<vector<int>(int,int)> getPath = [&](int start, int end) {
        if (nextHop[start][end] == -1) {
            return vector<int>();
        }
        vector<int> path;
        int cur = start;
        while (cur != end) {
            path.push_back(cur);
            cur = nextHop[cur][end];
            if (cur == -1) return vector<int>();
        }
        path.push_back(end);
        return path;
    };

    for (int i = 1; i <= nodeCount; i++) {
        for (int j = 1; j <= nodeCount; j++) {
            if (i == j) continue;
            
            if (dist[i][j].b >= INF && dist[i][j].a == 0.0) continue;
    
            vector<int> nodePath = getPath(i, j);

            shortest_path[i][j] = nodePath;
        }
    }


    // cout << "Finished populating kPaths!" << endl;

    // cout << kPaths[3][4].size() << "\n";
    // for(auto it: kPaths[3][4][2])
    // cout << it << " ";
    

//-========== Examples for debugging =====================================
    // double finalMsgSize = 50.0;
    // int src = 2, dst = 4;
    // Cost c = dist[src][dst];
    // if (c.b >= INF && c.a == 0.0) {
    //     cout << "No path from " << src << " to " << dst << "\n";
    // } else {
    //     double totalTime = costVal(c, finalMsgSize);
    //     cout << fixed << setprecision(4);
    //     cout << "Shortest path cost from " << src << " to " << dst
    //          << " for msgSize=" << finalMsgSize
    //          << " is " << totalTime << "\n";
    //     cout << "Links used:\n";
    //     for (auto node : shortest_path[src][dst]) {
    //         cout << "  NodeID=" << node << "\n";
    //     }
    // }


    // auto res = yenKShortestPaths(1, 4, 1, adjacency);
    // for (int i = 0; i < (int)res.size(); i++) {
    //     cout << i+1 << ") cost=" << res[i].cost << " path: ";
    //     for (int nd : res[i].nodes) {
    //         cout << nd << " ";
    //     }
    //     cout << "\n";
    // }
//========================================================================================================================

//-================ Taking care of the preloading==========================

    // cout << (*processorBusyMap[p1.id].begin()).start;
    reserveSlotForProcessor(p1.id,0,20,1);
    reserveSlotForProcessor(p1.id,50,10,1);
    reserveSlotForProcessor(p1.id,100,20,1);
    reserveSlotForProcessor(p1.id,150,10,1);
    reserveSlotForProcessor(p2.id,10,30,1);
    reserveSlotForProcessor(p2.id,110,30,1);
    reserveSlotForProcessor(p3.id,10,10,1);
    reserveSlotForProcessor(p3.id,60,20,1);
    reserveSlotForProcessor(p3.id,110,10,1);
    reserveSlotForProcessor(p3.id,180,20,1);
    reserveSlotForProcessor(p4.id,30,10,1);
    reserveSlotForProcessor(p4.id,80,20,1);
    reserveSlotForProcessor(p4.id,130,10,1);
    reserveSlotForProcessor(p4.id,180,20,1);
    
    

    reserveSlotForProcessor(p1.id,200,20,1);
    reserveSlotForProcessor(p1.id,250,10,1);
    reserveSlotForProcessor(p1.id,300,20,1);
    reserveSlotForProcessor(p1.id,350,10,1);
    reserveSlotForProcessor(p2.id,210,30,1);
    reserveSlotForProcessor(p2.id,310,30,1);
    reserveSlotForProcessor(p3.id,210,10,1);
    reserveSlotForProcessor(p3.id,260,20,1);
    reserveSlotForProcessor(p3.id,310,10,1);
    reserveSlotForProcessor(p3.id,380,20,1);
    reserveSlotForProcessor(p4.id,230,10,1);
    reserveSlotForProcessor(p4.id,280,20,1);
    reserveSlotForProcessor(p4.id,330,10,1);
    reserveSlotForProcessor(p4.id,380,20,1);


    

    linkSchedules[l1.id].reserveSlot(0,30);
    linkSchedules[l1.id].reserveSlot(170,30);
    linkSchedules[l2.id].reserveSlot(30,30);
    linkSchedules[l2.id].reserveSlot(140,30);
    linkSchedules[l3.id].reserveSlot(0,60);
    linkSchedules[l4.id].reserveSlot(140,60);
    linkSchedules[l5.id].reserveSlot(20,40);
    linkSchedules[l5.id].reserveSlot(80,20);
    linkSchedules[l5.id].reserveSlot(180,20);
    linkSchedules[l6.id].reserveSlot(70,60);
    linkSchedules[l7.id].reserveSlot(0,30);
    linkSchedules[l7.id].reserveSlot(170,30);
    linkSchedules[l8.id].reserveSlot(30,30);
    linkSchedules[l8.id].reserveSlot(140,30);

    linkSchedules[l1.id].reserveSlot(200,30);
    linkSchedules[l1.id].reserveSlot(370,30);
    linkSchedules[l2.id].reserveSlot(230,30);
    linkSchedules[l2.id].reserveSlot(340,30);
    linkSchedules[l3.id].reserveSlot(200,60);
    linkSchedules[l4.id].reserveSlot(340,60);
    linkSchedules[l5.id].reserveSlot(220,40);
    linkSchedules[l5.id].reserveSlot(280,20);
    linkSchedules[l5.id].reserveSlot(380,20);
    linkSchedules[l6.id].reserveSlot(270,60);
    linkSchedules[l7.id].reserveSlot(200,30);
    linkSchedules[l7.id].reserveSlot(370,30);
    linkSchedules[l8.id].reserveSlot(230,30);
    linkSchedules[l8.id].reserveSlot(340,30);





    computeAllRanks();
    computeAllPriority();

    // for(auto it: ranks){
    //     cout << it << " ";
    // }
    // for(auto it: priority){
    //     cout << it << " ";
    // }


    // for(int i=1;i<=tasks.size();i++){
    //     for(int k=1;k<=processors.size();k++){
    //         cout << rank2D[i][k] << "         ";
    //     }
    //     cout << "\n";
    // }
    

    // cout << costVal(dist[1][4],1.0);

    vector<int> dummyTaskList = createTaskList(); // has the task ids in order from index 0
    queue<int> taskList;
    for(auto it: dummyTaskList){
        taskList.push(it);
    }

    // for(auto it: dummyTaskList) cout << it << " ";

    int exit_task_id;
    while(!taskList.empty()){
        Task current_task = taskById[taskList.front()];
        vector<vector<double>> eft2D;
        vector<double> max_comm_time;
        unordered_map<int, unordered_map<int, LinkSchedule>> dummyLinkSchedule;
        for(auto it: processors){
            dummyLinkSchedule[it.id] = linkSchedules;
        }
        eft2D.assign(tasks.size()+1, vector<double>(processors.size()+1,INF));
        max_comm_time.assign(processors.size()+1,0);
        for(auto p: processors){
            if(current_task.predecessors.size()==0){
                double taskStartTime = reserveSlotForProcessor(p.id, 0, execution_times[current_task.id][p.id],0);
                eft2D[current_task.id][p.id] = taskStartTime + execution_times[current_task.id][p.id];
                // if(eft2D[current_task.id][p.id]<1e9) cout << "Can schedule task " << current_task.id <<  " on " << p.id <<  " starting at: " << eft2D[current_task.id][p.id] << endl;
            }
            else{
                vector<int> message_priority_list;
                message_priority_list = create_message_priority_list(current_task);
                max_comm_time[p.id] = schedule_messages_astar(current_task.id, p.id, message_priority_list, dummyLinkSchedule[p.id]);
                double taskStartTime = reserveSlotForProcessor(p.id, max_comm_time[p.id], execution_times[current_task.id][p.id],0);
                if(taskStartTime>=0){
                    // cout << "Can schedule task " << current_task.id <<  " on Processor " << p.id <<  " starting at: " << taskStartTime << endl;
                    eft2D[current_task.id][p.id] = taskStartTime + execution_times[current_task.id][p.id];
                }
                if(max_comm_time[p.id] < 0){
                    eft2D[current_task.id][p.id] = INF;
                    continue;
                }
            }
        }
        double minVal = INF;
        int chosen_proc_id, cnt=0;
        for(auto it: eft2D[current_task.id]){
            if(minVal>it){
                minVal = it;
                chosen_proc_id = cnt;
            }
            cnt++;
        }
        finishTimeOfTask[current_task.id] = minVal;
        assignedProcOfTask[current_task.id] = chosen_proc_id;
        cout << "Processor " << chosen_proc_id << " does Task " << current_task.id << " from " << finishTimeOfTask[current_task.id] - execution_times[current_task.id][chosen_proc_id] << " to " << finishTimeOfTask[current_task.id] << "\n";
        linkSchedules = dummyLinkSchedule[chosen_proc_id];
        reserveSlotForProcessor(chosen_proc_id, max_comm_time[chosen_proc_id], execution_times[current_task.id][chosen_proc_id],1);
        dummyLinkSchedule.clear();
        if(taskList.size()==1){
            exit_task_id = taskList.front();
        }
        taskList.pop();
    }
    

    cout << "\nMakespan: " << finishTimeOfTask[exit_task_id] << "\n";

    auto end = chrono::high_resolution_clock::now();

    chrono::duration<double> elapsed = end - start;

    cout << "Runtime: " << elapsed.count() << " \n";

    // int count;
    // for(auto p: processors){
    //     count = 0;
    //     for(auto it: processorBusyMap[p.id]){
    //         while(count<it.start && count<=hyperperiod_end){
    //             cout << " ";
    //             count++;
    //         }
    //         while(count<=it.end && count<=hyperperiod_end){
    //             cout << "l";
    //             count++;
    //         }
    //     }
    //     while(count<=hyperperiod_end){
    //         cout << " ";
    //         count++;
    //     }
    //     cout << "\n";
    // }

    cout << "\nProcessor Schedules:\n";
    for(auto p: processors){
        for(auto it: processorBusyMap[p.id]){
            cout << it.start << " " << it.end << "    ";
        }
        cout << "\n";
    }

    cout << "\nLink Schedules:\n";
    for(auto l: links){
        for(auto it: linkSchedules[l.id].busyIntervals){
            cout << it.departure_start << " " << it.departure_end << "   ";
        }
        cout << "\n";
    }
    return 0;
}


