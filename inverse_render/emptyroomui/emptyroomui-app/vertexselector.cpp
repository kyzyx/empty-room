#include "vertexselector.h"
#include <algorithm>

using namespace std;

void VectorVertexSelector::selected(vector<int>& selected)
{
    if (!is_sorted(selected.begin(), selected.end())) sort(selected.begin(), selected.end());
    if (selectmode == SELECTMODE_ADD) {
        vector<int> tmp;
        set_union(indices.begin(), indices.end(), selected.begin(), selected.end(), tmp.begin());
        swap(tmp, indices);
    } else if (selectmode == SELECTMODE_REMOVE) {
        vector<int> tmp;
        set_difference(indices.begin(), indices.end(), selected.begin(), selected.end(), tmp.begin());
        swap(tmp, indices);
    } else if (selectmode == SELECTMODE_REPLACE) {
        indices = selected;
    }
}


void SetVertexSelector::selected(vector<int>& selected)
{
    switch(selectmode) {
        case SELECTMODE_REPLACE:
            indices.clear();
        case SELECTMODE_ADD:
            for (int i = 0; i < selected.size(); i++) indices.insert(selected[i]);
            break;
        case SELECTMODE_REMOVE:
            for (int i = 0; i < selected.size(); i++) indices.erase(selected[i]);
            break;
    }
}

int SetVertexSelector::operator [](int n) {
    if (idx > n) {
        if (idx-n < n) {
            while (idx > n) {
                it--;
                idx--;
            }
        } else {
            it = indices.cbegin();
        }
    }
    while (idx < n) {
        it++;
        idx++;
    }
    return *it;
}
