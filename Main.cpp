#include <bits/stdc++.h>
using namespace std;

struct Pos { int r,c; bool operator==(Pos const& o) const { return r==o.r && c==o.c; } };
struct PosHash { size_t operator()(Pos const& p) const noexcept { return (p.r*1315423911u) ^ (p.c<<16); } };
using Path = vector<Pos>;

struct Grid {
    int rows, cols;
    vector<char> cells;
    Grid(int r=0,int c=0):rows(r),cols(c),cells(r*c,0){}
    bool inBounds(Pos p) const { return p.r>=0 && p.r<rows && p.c>=0 && p.c<cols; }
    bool isFree(Pos p) const { return inBounds(p) && cells[p.r*cols+p.c]==0; }
    void setObstacle(Pos p){ if(inBounds(p)) cells[p.r*cols+p.c]=1; }
    int id(Pos p) const { return p.r*cols + p.c; }
    Pos fromId(int id) const { return {id/cols,id%cols}; }
};

struct DynamicObstacle {
    Path schedule;
    Pos at(int t) const {
        if(schedule.empty()) return {-1,-1};
        if(t < (int)schedule.size()) return schedule[t];
        return schedule.back();
    }
};

struct Reservation {
    unordered_map<long long,int> vertex;
    unordered_set<long long> edges;
    static long long keyVertex(Pos p,int t){ return (((long long)t)<<32) | (long long)(p.r<<16) | p.c; }
    static long long keyEdge(Pos a,Pos b,int t){ return (((long long)t)<<48) ^ (((long long)a.r)<<36) ^ (((long long)a.c)<<24) ^ (((long long)b.r)<<12) ^ (long long)b.c; }
    void reserveVertex(Pos p,int t,int id){ vertex[keyVertex(p,t)] = id; }
    void reserveEdge(Pos a,Pos b,int t){ edges.insert(keyEdge(a,b,t)); }
    bool conflictVertex(Pos p,int t) const { return vertex.find(keyVertex(p,t))!=vertex.end(); }
    bool conflictEdge(Pos a,Pos b,int t) const { return edges.find(keyEdge(a,b,t))!=edges.end(); }
};

struct AStarResult { Path path; int nodesExpanded; bool success; };

struct AStarPlanner {
    const Grid& grid;
    const Reservation* baseRes;
    vector<DynamicObstacle>* dynObs;
    int maxT;
    AStarPlanner(const Grid& g,const Reservation* r, vector<DynamicObstacle>* d,int horizon):grid(g),baseRes(r),dynObs(d),maxT(horizon){}
    int heuristic(Pos a, Pos b){ return abs(a.r-b.r)+abs(a.c-b.c); }
    AStarResult plan(Pos start, Pos goal, int agentId){
        struct Node { Pos p; int t; int g; int f; };
        struct NodeCmp { bool operator()(Node const& a, Node const& b) const { if(a.f!=b.f) return a.f>b.f; return a.g<b.g; } };
        priority_queue<Node, vector<Node>, NodeCmp> pq;
        unordered_map<long long, int> gscore;
        auto key=[&](Pos p,int t)->long long{ return (((long long)t)<<32) | (long long)(p.r<<16) | p.c; };
        auto freeAt=[&](Pos p,int t)->bool{
            if(!grid.isFree(p)) return false;
            if(baseRes && baseRes->conflictVertex(p,t)) return false;
            for(auto &d : *dynObs){
                Pos dp = d.at(t);
                if(dp==p) return false;
            }
            return true;
        };
        if(!freeAt(start,0)) return {Path(),0,false};
        Node s{start,0,0,heuristic(start,goal)};
        pq.push(s);
        gscore[key(start,0)] = 0;
        int expanded=0;
        vector<Pos> dirs = {{0,1},{1,0},{0,-1},{-1,0},{0,0}};
        while(!pq.empty()){
            Node cur = pq.top(); pq.pop();
            expanded++;
            if(cur.t>maxT) continue;
            if(cur.p==goal){
                Path path;
                long long kk = key(cur.p,cur.t);
                unordered_map<long long, pair<long long,Pos>> came;
                break;
            }
            for(auto d:dirs){
                Pos np{cur.p.r + d.r, cur.p.c + d.c};
                int nt = cur.t + 1;
                if(nt>maxT) continue;
                if(!grid.inBounds(np)) continue;
                if(!freeAt(np,nt)) continue;
                if(baseRes){
                    if(baseRes->conflictEdge(cur.p,np,cur.t)) continue;
                }
                bool swapConflict=false;
                for(auto &dof : *dynObs){
                    Pos prev = dof.at(cur.t);
                    Pos now = dof.at(nt);
                    if(prev==np && now==cur.p) swapConflict=true;
                }
                if(swapConflict) continue;
                long long nk = key(np,nt);
                int ng = cur.g + 1;
                if(gscore.find(nk)==gscore.end() || ng < gscore[nk]){
                    gscore[nk]=ng;
                    int f = ng + heuristic(np,goal);
                    pq.push({np,nt,ng,f});
                }
            }
        }
        unordered_map<long long, int> g2;
        unordered_map<long long, long long> parent;
        auto pushNode=[&](Pos p,int t,int g,int f, unordered_map<long long,int>& gmap, priority_queue<Node, vector<Node>, NodeCmp>& q){
            long long k=key(p,t);
            gmap[k]=g;
            q.push({p,t,g,f});
        };
        while(!pq.empty()) pq.pop();
        g2.clear(); parent.clear();
        pushNode(start,0,0,heuristic(start,goal), g2, pq);
        expanded=0;
        while(!pq.empty()){
            Node cur=pq.top(); pq.pop();
            expanded++;
            long long ck=key(cur.p,cur.t);
            if(g2[ck] < cur.g) continue;
            if(cur.p==goal){
                Path path;
                long long curk=ck;
                while(true){
                    int t = (int)(curk>>32);
                    int rc = (int)(curk & 0xffffffff);
                    Pos p = {(rc>>16)&0xffff, rc&0xffff};
                    path.push_back(p);
                    if(t==0 && p==start) break;
                    if(parent.find(curk)==parent.end()) break;
                    curk = parent[curk];
                }
                reverse(path.begin(), path.end());
                return {path, expanded, true};
            }
            for(auto d:dirs){
                Pos np{cur.p.r + d.r, cur.p.c + d.c};
                int nt = cur.t + 1;
                if(nt>maxT) continue;
                if(!grid.inBounds(np)) continue;
                if(!freeAt(np,nt)) continue;
                if(baseRes && baseRes->conflictEdge(cur.p,np,cur.t)) continue;
                bool swapConflict=false;
                for(auto &dof : *dynObs){
                    Pos prev = dof.at(cur.t);
                    Pos now = dof.at(nt);
                    if(prev==np && now==cur.p) swapConflict=true;
                }
                if(swapConflict) continue;
                long long nk=key(np,nt);
                int ng = cur.g + 1;
                if(g2.find(nk)==g2.end() || ng < g2[nk]){
                    g2[nk]=ng;
                    parent[nk]=(((long long)cur.t)<<32) | (long long)(cur.p.r<<16) | cur.p.c;
                    int f = ng + heuristic(np,goal);
                    pq.push({np,nt,ng,f});
                }
            }
        }
        return {Path(),expanded,false};
    }
};

struct Simulator {
    Grid grid;
    vector<DynamicObstacle> dyn;
    vector<pair<Pos,Pos>> robots;
    int horizon;
    Reservation globalRes;
    Simulator(int r,int c,int h):grid(r,c),horizon(h){}
    void addRack(Pos p){ grid.setObstacle(p); }
    void addDynamicObstacle(const Path& p){ dyn.push_back({p}); }
    void addRobot(Pos s,Pos g){ robots.push_back({s,g}); }
    void reserveDynamics(){
        for(int t=0;t<=horizon;t++){
            for(auto &d:dyn){
                Pos p = d.at(t);
                if(p.r>=0) globalRes.reserveVertex(p,t,-1);
            }
        }
    }
    vector<Path> prioritizedPlanning(bool useReplan, int &totalNodes, double &timeTaken){
        reserveDynamics();
        vector<Path> plans(robots.size());
        totalNodes=0;
        auto t0 = chrono::high_resolution_clock::now();
        for(int i=0;i<(int)robots.size();i++){
            AStarPlanner planner(grid,&globalRes,&dyn,horizon);
            auto res = planner.plan(robots[i].first, robots[i].second, i);
            totalNodes += res.nodesExpanded;
            if(!res.success && useReplan){
                int extraH = horizon*2;
                AStarPlanner repl(grid,&globalRes,&dyn,extraH);
                res = repl.plan(robots[i].first, robots[i].second, i);
            }
            if(!res.success) res.path = {robots[i].first};
            plans[i] = res.path;
            for(int t=0;t<(int)plans[i].size();t++){
                globalRes.reserveVertex(plans[i][t], t, i);
                if(t>0) globalRes.reserveEdge(plans[i][t-1], plans[i][t], t-1);
            }
            if(plans[i].size()>0){
                Pos last = plans[i].back();
                for(int t=plans[i].size(); t<=horizon; t++) globalRes.reserveVertex(last,t,i);
            }
        }
        auto t1 = chrono::high_resolution_clock::now();
        timeTaken = chrono::duration_cast<chrono::duration<double>>(t1-t0).count();
        return plans;
    }
    void simulateRun(bool replan){
        int nodes=0; double timeTaken=0;
        Reservation saved = globalRes;
        vector<Path> plans = prioritizedPlanning(replan,nodes,timeTaken);
        cout<<"Planning nodes expanded: "<<nodes<<"\n";
        cout<<"Planning time secs: "<<timeTaken<<"\n";
        for(int i=0;i<(int)plans.size();i++){
            cout<<"Robot "<<i<<": ";
            for(auto &p:plans[i]) cout<<"("<<p.r<<","<<p.c<<")->";
            cout<<"END\n";
        }
        globalRes = saved;
    }
};

int main(){
    int R=12, C=16, H=200;
    Simulator sim(R,C,H);
    vector<Pos> racks = {{1,2},{1,3},{1,4},{1,5},{2,5},{3,5},{4,5},{5,5},{6,5},{7,5},{8,5},{9,5},{10,5},{10,6},{10,7},{10,8}};
    for(auto p:racks) sim.addRack(p);
    Path w1;
    for(int t=0;t<40;t++) w1.push_back({2+t%5, 9});
    sim.addDynamicObstacle(w1);
    Path w2;
    for(int t=0;t<40;t++) w2.push_back({8, 2+(t%6)});
    sim.addDynamicObstacle(w2);
    sim.addRobot({0,0},{11,15});
    sim.addRobot({0,2},{10,14});
    sim.addRobot({3,0},{9,13});
    sim.addRobot({11,0},{0,15});
    sim.addRobot({5,1},{5,14});
    sim.simulateRun(true);
    return 0;
}
