#include <bits/stdc++.h>
using namespace std;


//--------------------- Structs required for Computation Model -----------------------------------------------------------------------------
struct Task{
    int id;
    int computation_units;
    vector<int> successors;
    vector<int> predecessors;
    Task(int id_, int computation_units_)
        : id(id_), computation_units(computation_units_){}
};

struct Message {
    int id;
    int srcTaskId;
    int dstTaskId;
    int size;

    Message(int id_, int src, int dst, double size_)
        : id(id_), srcTaskId(src), dstTaskId(dst), size(size_) {}
};




//--------------------- Structs required for Platform Model ---------------------------------------------------------------------------------
struct Processor {
    int id;
    double speed;
    double startup_delay;

    Processor(int id_, double speed_, double startup_delay_)
        : id(id_), speed(speed_), startup_delay(startup_delay_) {}

};

struct Link {
    int id;
    int endpoint1;
    int endpoint2;
    double bandwidth;

    Link(int id_, int ep1, int ep2, double bandwidth_)
        : id(id_), endpoint1(ep1), endpoint2(ep2), bandwidth(bandwidth_) {}
};

struct GateControlEntry {
    int output_link_id;
    double departure_start;
    double departure_end;
    
    GateControlEntry(int out, double start, double end)
        : output_link_id(out), departure_start(start), departure_end(end) {}
};

struct GateControlEntryComparator {
    bool operator()(const GateControlEntry& a, const GateControlEntry& b) const {
        return a.departure_start < b.departure_start;
    }
};

struct Switch {
    int id;
    double startup_delay;
    
    unordered_map<int, multiset<GateControlEntry, GateControlEntryComparator>> gateControlMap;
    
    Switch(int id_, double startup_delay_)
        : id(id_), startup_delay(startup_delay_) {}
    
    void addGateControlEntry(int link_id, double departure_start, double departure_end) {
        GateControlEntry entry(link_id, departure_start, departure_end);
        gateControlMap[link_id].insert(entry);
    }
    
    double reserveSlotForLink(int link_id, double arrival_time, double required_duration, int dummy_link_id, double hyperperiod_end) {
        double free_start = arrival_time;
        
        if (free_start + required_duration > hyperperiod_end)
            return -1;
        
        if (gateControlMap.find(link_id) != gateControlMap.end()) {
            for (const auto &entry : gateControlMap[link_id]) {
                if (free_start + required_duration <= entry.departure_start) {
                    addGateControlEntry(dummy_link_id, free_start, free_start + required_duration);
                    return free_start;
                }
                if (free_start >= entry.departure_start && free_start < entry.departure_end) {
                    free_start = entry.departure_end;
                }
                if (free_start + required_duration > hyperperiod_end)
                    return -1;
            }
        }
        
        addGateControlEntry(dummy_link_id, free_start, free_start + required_duration); //ponder more on this dummy link id idea, we are always assigning to dummy so make it 
                                                                                        //permanent and then also clear dummies
        return free_start;
    }
};
//----------------------------------------------------------------------------------------------------------------------------------------------

int main()
{
    ios::sync_with_stdio(0);
    cin.tie(0);

//--------Creation of Computation Model-----------------
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
//------------------------------------------------------

//--------Creation of Platform Model--------------------
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

    vector<Link> links = {l1,l2,l3,l4,l5,l6,l7,l8};

}