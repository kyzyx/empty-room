#include "findplanes.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

const double ANGLETHRESHOLD = M_PI/8;
const double DISTTHRESHOLD = 0.03;
const double NOISETHRESHOLD = 0.01;
const double MININLIERPROPORTION = 0.05;
const double MAXEDGEPROPORTION = 0.04;

using namespace std;
using namespace Eigen;
using namespace pcl;

inline bool onPlane(Vector4d plane, PointXYZ p) {
    return abs(p.x*plane(0) + p.y*plane(1) + p.z*plane(2) + plane(3)) < NOISETHRESHOLD;
}
double calculateEdgeProportion(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, vector<int> indices)
{
    vector<uint8_t> labels(cloud->size(), -1);
    for (int i = 0; i < indices.size(); ++i) {
        int x = indices[i]%cloud->width;
        int y = indices[i]/cloud->width;

        labels[indices[i]] = 0;
        if (x != cloud->width-1 && labels[indices[i]+1])
            labels[indices[i]+1] = 1;
        if (x && labels[indices[i]-1])
            labels[indices[i]-1] = 1;
        if (y != cloud->height-1 && labels[indices[i]+cloud->width])
            labels[indices[i]+cloud->width] = 1;
        if (y && labels[indices[i]-cloud->width])
            labels[indices[i]-cloud->width] = 1;
    }
    int count = 0;
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] == 1) ++count;
    }
    return count/(double) indices.size();
}
void findPlanesRANSAC(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    ModelCoefficients::Ptr coef(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices);
    SACSegmentation<PointXYZL> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(2*NOISETHRESHOLD);
    seg.setMaxIterations(100);
    ExtractIndices<PointXYZL> extract;

    PointCloud<PointXYZL>::Ptr filtered(new PointCloud<PointXYZL>);
    copyPointCloud(*cloud, *filtered);
    for (int i = 0; i < filtered->size(); ++i) {
        filtered->at(i).label = i;
    }

    int n = 0;
    for (int z = 0; z < 10 && filtered->size() > MININLIERPROPORTION*cloud->size(); ++z) {
        // Find planes, if any
        seg.setInputCloud(filtered);
        seg.segment(*inliers, *coef);
        if (inliers->indices.size() == 0) {
            break;
        }
        // Filter planes
        // A plane is valid if it takes up a large proportion of the points
        // in the cloud, and if it has many large contiguous sections.
        if (inliers->indices.size() > MININLIERPROPORTION*cloud->size()) {
            vector<int> originalindices;
            for (int i = 0; i < inliers->indices.size(); ++i) {
                originalindices.push_back(filtered->at(inliers->indices[i]).label);
            }
            double edgeprop = calculateEdgeProportion(cloud, originalindices);
            if (edgeprop < MAXEDGEPROPORTION) {
                // Add plane to list
                for (int i = 0; i < originalindices.size(); ++i) {
                    ids[originalindices[i]] = n;
                }
                Vector4d p(coef->values[0], coef->values[1], coef->values[2], coef->values[3]);
                planes.push_back(p);
                // Remove points on plane
                extract.setInputCloud(filtered);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*filtered);
                n++;
            }
        }
    }
}

void findPlanesRANSACWithNormals(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    ModelCoefficients::Ptr coef(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices);
    SACSegmentationFromNormals<PointXYZINormal, PointXYZINormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_NORMAL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(NOISETHRESHOLD);
    seg.setMaxIterations(100);
    seg.setEpsAngle(ANGLETHRESHOLD);
    ExtractIndices<PointXYZINormal> extract;

    PointCloud<PointXYZINormal>::Ptr filtered(new PointCloud<PointXYZINormal>);
    copyPointCloud(*cloud, *filtered);
    // HACK: Use intensity (float field) as an id (int) field
    for (int i = 0; i < filtered->size(); ++i) {
        int* I = (int*) &(filtered->at(i).intensity);
        *I = i;
    }
    IntegralImageNormalEstimation<PointXYZ, PointXYZINormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*filtered);

    int n = 0;
    while (filtered->size() > 0.2*cloud->size()) {
        // Find planes, if any
        seg.setInputCloud(filtered);
        seg.setInputNormals(filtered);
        seg.segment(*inliers, *coef);
        if (inliers->indices.size() == 0) {
            break;
        }

        // Add plane to list
        for (int i = 0; i < inliers->indices.size(); ++i) {
            int* I = (int*) &(filtered->at(inliers->indices[i]).intensity);
            int idx = *I;
            ids[idx] = n;
        }
        Vector4d p(coef->values[0], coef->values[1], coef->values[2], coef->values[3]);
        planes.push_back(p);

        // Remove points on plane
        extract.setInputCloud(filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*filtered);
        n++;
    }
}

inline Vector3d getNormal(PointNormal p) {
    return Vector3d(p.normal_x, p.normal_y, p.normal_z);
}

static DefaultPointRepresentation<PointXYZ> pr;
inline double dist2(PointXYZ a, PointXYZ b) {
    if (!pr.isValid(a) || !pr.isValid(b)) return numeric_limits<double>::infinity();
    return (Vector3f(a.x,a.y,a.z)-Vector3f(b.x,b.y,b.z)).squaredNorm();
}

void findPlanesWithNormals(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    if (cloud->height == 1) {
        cerr << "Error: findPlanesWithNormals expects a range image" << endl;
        return;
    }
    PointCloud<PointNormal>::Ptr filtered(new PointCloud<PointNormal>);
    copyPointCloud(*cloud, *filtered);
    IntegralImageNormalEstimation<PointXYZ, PointNormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*filtered);

    // BFS to find components in image
    int n = 0;
    vector<Vector4d> candidates;
    vector<int> candidateids;
    vector<int> visited(ids);
    vector<pair<int,int> > sizes;
    for (int i = 0; i < cloud->height-1; ++i) {
        for (int j = 0; j < cloud->width-1; ++j) {
            if (visited[i*cloud->width + j] >= 0) continue;
            queue<int> nx; queue<int> ny;
            nx.push(j); ny.push(i);
            Vector3d avgv(0,0,0);
            int cnt = 0;
            while (!nx.empty()) {
                int x = nx.front(); nx.pop();
                int y = ny.front(); ny.pop();
                if (!pr.isValid(cloud->at(x,y)) || isnan(filtered->at(x,y).normal_x)) continue;
                visited[y*cloud->width + x] = n;
                avgv = (avgv*cnt + getNormal(filtered->at(x,y)))/(cnt+1);
                avgv.normalize();
                ++cnt;
                for (int xx = x-1; xx <= x+1; ++xx) {
                    if (xx < 0 || xx >= cloud->width-1) continue;
                    int t = xx==x?1:0;
                    for (int yy = y-t; yy <= y+t; ++yy) {
                        if (yy < 0 || yy >= cloud->height-1) continue;
                        if (visited[yy*cloud->width+xx] != -1) continue;
                        // Normal matching check
                        if (getNormal(filtered->at(xx,yy)).dot(avgv) > cos(ANGLETHRESHOLD)) {
                            // Close together check
                            // if (dist2(cloud->at(x,y), cloud->at(xx,yy)) < DISTTHRESHOLD)
                            // Coplanar check
                            Vector3d v(
                                    cloud->at(x,y).x - cloud->at(xx,yy).x,
                                    cloud->at(x,y).y - cloud->at(xx,yy).y,
                                    cloud->at(x,y).z - cloud->at(xx,yy).z);
                            v.normalize();
                            if (v.dot(avgv) < sin(ANGLETHRESHOLD)) {
                                nx.push(xx); ny.push(yy);
                                visited[yy*cloud->width+xx] = -2;
                            }
                        }
                    }
                }
            }
            //if (cnt/(double) cloud->size() > MININLIERPROPORTION)
            if (cnt > 10000) {
                candidates.push_back(Vector4d(avgv(0), avgv(1), avgv(2), 0));
                candidateids.push_back(n);
                sizes.push_back(make_pair(cnt, sizes.size()));
            }
            n++;
        }
    }

    for (int i = 0; i < cloud->size(); ++i) {
        for (int j = 0; j < candidates.size(); ++j) {
            if (visited[i] == candidateids[j]) {
                candidates[j](3) -= candidates[j](0)*cloud->at(i).x
                              + candidates[j](1)*cloud->at(i).y
                              + candidates[j](2)*cloud->at(i).z;
            }
        }
    }
    for (int i = 0; i < candidates.size(); ++i) {
        candidates[i](3) /= sizes[i].first;
    }
    sort(sizes.begin(), sizes.end(), greater<pair<int, int> >());
    vector<int> tmpids(candidateids);
    for (int i = 0; i < sizes.size(); ++i) {
        planes.push_back(candidates[sizes[i].second]);
        candidateids[i] = tmpids[sizes[i].second];
    }

    for (int i = 0; i < cloud->size(); ++i) {
        for (int j = 0; j < planes.size(); ++j) {
            if (visited[i] == candidateids[j]) {
                ids[i] = j;
                break;
            }
        }
        if (ids[i] == -1) {
            for (int j = 0; j < planes.size(); ++j) {
                if (!pr.isValid(cloud->at(i)) || !onPlane(planes[j], cloud->at(i))) continue;
                if (pr.isValid(cloud->at(i)) && isnan(filtered->at(i).normal_x)) {
                    // If null normal, it's good enough to just be on the plane
                    ids[i] = j;
                } else {
                    Vector3d norm = planes[j].head(3);
                    if (getNormal(filtered->at(i)).dot(norm) > cos(2*ANGLETHRESHOLD)) {
                        ids[i] = j;
                    }
                }
            }
        }
    }
}

void findPlanesManhattan(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
}

void findPlanesGradient(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    if (cloud->height == 1) {
        cerr << "Error: findPlanesGradient expects a range image" << endl;
        return;
    }
    // Calculate gradients
    double** dx = new double*[cloud->height];
    double** dy = new double*[cloud->height];
    for (int i = 0; i < cloud->height; ++i) {
        dx[i] = new double[cloud->width];
        dy[i] = new double[cloud->width];

        for (int j = 0; j < cloud->width; ++j) {
            if (j < cloud->width-1) dx[i][j] = cloud->at(j+1,i).z - cloud->at(j,i).z;
            if (i < cloud->height-1) dy[i][j] = cloud->at(j,i+1).z - cloud->at(j,i).z;
        }
    }
    // BFS to find components in image
    int n = 0;
    vector<int> sizes;
    DefaultPointRepresentation<PointXYZ> pr;
    /*for (int i = 0; i < cloud->height-1; ++i) {
        for (int j = 0; j < cloud->width-1; ++j) {
            if (ids[i*cloud->width + j] >= 0) continue;
            queue<int> nx; queue<int> ny;
            nx.push(j); ny.push(i);
            double adx = 0;
            double ady = 0;
            int cnt = 0;
            while (!nx.empty()) {
                int x = nx.top(); nx.pop();
                int y = ny.top(); ny.pop();
                if (!pr.isValid(cloud->at(x,y))) continue;
                ids[y*cloud->width + x] = n;
                adx = (adx*cnt + dx[x][y])/(cnt+1);
                ady = (ady*cnt + dy[x][y])/(cnt+1);
                ++cnt;
                for (int xx = x-1; xx <= x+1; ++xx) {
                    if (xx < 0 || xx >= cloud->width-1) continue;
                    int t = xx?0:1;
                    for (int yy = y-t; yy <= y+t; ++yy) {
                        if (yy < 0 || yy >= cloud->height-1) continue;
                        if (t) { // Check y gradients
                            if (abs(dy[xx][yy] - ady) > DGRAD) continue;
                        } else { // Check x gradients
                            if (!yy) continue;
                            if (abs(dx[xx][yy] - adx) > DGRAD) continue;
                        }
                        nx.push(xx); ny.push(yy);
                    }
                }
            }
            if (cnt) {
                sizes.push_back(cnt);
                n++;
            }
        }
    }*/

    // Global consistency
    for (int i = 0; i < n; ++i) {

    }

    // Filter out small components
    int nremoved = 0;
    for (int i = 0; i < n; ++i) {
        if (sizes[i+nremoved]/(double) cloud->size() < MININLIERPROPORTION) {
            nremoved++;
            for (int j = 0; j < ids.size(); ++j) {
                if (ids[j] == i) ids[j] = -1;
                else if (ids[j] > i) ids[j]--;
            }
            --i;
        }
    }
}

void combineLikePlanes(PointCloud<PointXYZ>::ConstPtr cloud, vector<Vector4d>& planes, vector<int>& ids)
{
    vector<int> relabel(planes.size(),-1);
    vector<Vector4d> origplanes(planes);
    planes.clear();
    int n = 0;
    for (int i = 0; i < origplanes.size(); ++i) {
        if (relabel[i] == -1) {
            relabel[i] = n;
            planes.push_back(origplanes[i]);
            for (int j = i+1; j < origplanes.size(); ++j) {
                if (origplanes[i].head(3).dot(origplanes[j].head(3)) > cos(ANGLETHRESHOLD)) {
                    if (abs(origplanes[i](3) - origplanes[j](3)) < NOISETHRESHOLD) {
                        relabel[j] = n;
                    }
                }
            }
            ++n;
        }
    }
    vector<int> origcounts(origplanes.size(),0);
    vector<int> counts(planes.size(),0);
    vector<Vector3d> norms(planes.size(), Vector3d::Zero());
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] >= 0) {
            origcounts[ids[i]]++;
            ids[i] = relabel[ids[i]];
            counts[ids[i]]++;
        }
    }
    for (int i = 0; i < relabel.size(); ++i) {
        if (relabel[i] >= 0)  {
            norms[relabel[i]] += origcounts[i]*origplanes[i].head(3);
        }
    }
    for (int i = 0; i < planes.size(); ++i) {
        planes[i].head(3) = norms[i].normalized();
        planes[i](3) = 0;
    }
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] >= 0) planes[ids[i]](3) -= Vector3d(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z).dot(planes[ids[i]].head(3));
    }
    for (int i = 0; i < planes.size(); ++i) {
        planes[i](3) /= counts[i];
    }
}

void findPlanes(
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids)
{
    ids.resize(cloud->size());
    for (int i = 0; i < ids.size(); ++i) ids[i] = -1;
    findPlanesWithNormals(cloud, planes, ids);
    combineLikePlanes(cloud, planes, ids);
    combineLikePlanes(cloud, planes, ids);
}
