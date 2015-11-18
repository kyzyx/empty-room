#ifndef VERTEXSELECTOR_H
#define VERTEXSELECTOR_H
#include <vector>
#include <set>

class VertexSelector
{
public:
    VertexSelector() : selectmode(SELECTMODE_ADD) {;}

    virtual void selected(std::vector<int>& selected) = 0;
    virtual void undo() {;}
    virtual void redo() {;}

    virtual void clear() = 0;
    virtual int size() const = 0;
    virtual int operator [](int n)  = 0;

    enum {
        SELECTMODE_ADD,
        SELECTMODE_REMOVE,
        SELECTMODE_REPLACE,
        NUMSELECTMODES
    };

    void setSelectMode(int n) {
        selectmode = n;
    }
    void setSelectModeAdd() {
        setSelectMode(SELECTMODE_ADD);
    }
    void setSelectModeRemove() {
        setSelectMode(SELECTMODE_REMOVE);
    }
    void setSelectModeReplace() {
        setSelectMode(SELECTMODE_REPLACE);
    }

protected:
    int selectmode;
};

class VectorVertexSelector : public VertexSelector
{
public:
    VectorVertexSelector() {;}

    virtual void selected(std::vector<int>& selected) override;

    virtual void clear() override { indices.clear(); }
    virtual int size() const override { return indices.size(); }
    virtual int operator [](int n) override { return indices[n]; }
protected:
    std::vector<int> indices;
};


class SetVertexSelector : public VertexSelector
{
public:
    SetVertexSelector() : idx(0) { it = indices.cbegin();}

    virtual void selected(std::vector<int>& selected) override;

    virtual void clear() override { indices.clear(); }
    virtual int size() const override { return indices.size(); }
    virtual int operator [](int n) override;
protected:
    std::set<int> indices;
    std::set<int>::const_iterator it;
    int idx;
};

#endif // VERTEXSELECTOR_H
