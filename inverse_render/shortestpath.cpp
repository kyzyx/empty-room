#include <algorithm>
#include <queue>
#include <limits>
#include <iostream>

using namespace std;

typedef pair<double,pair<int,int> > tdii;
double shortestPath(int from, int to, double** edges, int n, vector<int>& path) {
    if (from == to) {
        int bestn = -1;
        double best = numeric_limits<double>::max();
        double currcost;
        for (int i = 0; i < n; ++i) {
            if (i == from) continue;
            if (i < from) currcost = edges[from][i];
            else if (i > from) currcost = edges[i][from];
            if (currcost < numeric_limits<double>::max()) {
                if (i < from) edges[from][i] = numeric_limits<double>::max();
                else if (i > from) edges[i][from] = numeric_limits<double>::max();
                double c = shortestPath(from, i, edges, n, path) + currcost;
                if (c < best) {
                    best = c;
                    bestn = i;
                }
                if (i < from) edges[from][i] = currcost;
                else if (i > from) edges[i][from] = currcost;
            }
        }
        if (bestn < from) {
            currcost = edges[from][bestn];
            edges[from][bestn] = numeric_limits<double>::max();
        } else if (bestn > from) {
            currcost = edges[bestn][from];
            edges[bestn][from] = numeric_limits<double>::max();
        }
        shortestPath(from, bestn, edges, n, path);
        if (bestn < from) edges[from][bestn] = currcost;
        else if (bestn > from) edges[bestn][from] = currcost;
        return best;
    }
    vector<double> cost(n, numeric_limits<double>::max());
    vector<int> pred(n, -1);
    cost[from] = 0;
    priority_queue<tdii, vector<tdii>, greater<tdii> > next;
    int curr = from;
    path.clear();
    while(1) {
        for (int i = 0; i < n; ++i) {
            if (i == curr || i == pred[curr] || cost[i] < numeric_limits<double>::max()) continue;
            double currcost, costsofar;
            if (i > curr) {
                currcost = edges[i][curr];
            } else if (i < curr) {
                currcost = edges[curr][i];
            }
            if (currcost < numeric_limits<double>::max()) {
                next.push(make_pair(cost[curr]+currcost,make_pair(curr,i)));
            }
        }
        double c;
        int previd, nextid = -1;
        while(!next.empty()) {
            tdii t = next.top(); next.pop();
            c = t.first;
            previd = t.second.first;
            int nid = t.second.second;
            if (cost[nid] == numeric_limits<double>::max()) {
                nextid = nid;
                break;
            }
        }
        if (nextid == -1) break;
        curr = nextid;
        cost[curr] = c;
        pred[curr] = previd;
        if (curr == to) break;
    }
    if (curr == to) {
        while (curr != -1) {
            path.push_back(curr);
            curr = pred[curr];
        }
        reverse(path.begin(), path.end());
        return cost[to];
    } else {
        return numeric_limits<double>::max();
    }
}

