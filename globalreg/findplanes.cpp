#include "findplanes.h"
#include "util.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/SVD>

double ANGLETHRESHOLD = M_PI/9;
const double DISTTHRESHOLD = 0.03;
const double NOISETHRESHOLD = 0.01;
const double FUSETHRESHOLD = 0.05;
const double MININLIERPROPORTION = 0.05;
const double MAXEDGEPROPORTION = 0.04;
int MININLIERCOUNT = 8000;
const int MINPLANESIZE = 10000;
const double ANGULARDEVIATIONTHRESHOLD = 0.25;

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

bool isBehind(PointXYZ p, Vector4d plane) {
    if (!isValid(p)) return false;
    return (p.x*plane(0) + p.y*plane(1) + p.z*plane(2) + plane(3)) < -2*NOISETHRESHOLD;
}

int calculateBoundingSides(Vector4d plane, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, vector<int> indices, int index)
{
    const int windowsize = 25;
    const int windowprop = 10;
    const double sideprop = 0.1;
    vector<int> lines(4,0);
    vector<int> failed(4,0);
    // Check horizontal bounding-ness
    for (int i = 0; i < cloud->height; ++i) {
        for (int j = 0; j < cloud->width; ++j) {
            int idx = j + i*cloud->width;
            if (indices[idx] == index) {
                ++lines[0];
                if (j >= windowsize) {
                    int count = 0;
                    for (int k = 1; k < windowsize+1; ++k) {
                        if (isBehind(cloud->at(idx-k), plane)) {
                            ++count;
                        }
                    }
                    if (count >= windowprop) ++failed[0];
                }
                break;
            }
        }
        for (int j = cloud->width-1; j >= 0; --j) {
            int idx = j + i*cloud->width;
            if (indices[idx] == index) {
                ++lines[1];
                if (j < cloud->width - windowsize) {
                    int count = 0;
                    for (int k = 1; k < windowsize+1; ++k) {
                        if (isBehind(cloud->at(idx+k), plane)) {
                            ++count;
                        }
                    }
                    if (count >= windowprop) ++failed[1];
                }
                break;
            }
        }
    }
    // Check vertical bounding-ness
    for (int j = 0; j < cloud->width; ++j) {
        for (int i = 0; i < cloud->height; ++i) {
            int idx = j + i*cloud->width;
            if (indices[idx] == index) {
                ++lines[2];
                if (i >= windowsize) {
                    int count = 0;
                    for (int k = 1; k < windowsize+1; ++k) {
                        if (isBehind(cloud->at(idx-k*cloud->width), plane)) {
                            ++count;
                        }
                    }
                    if (count >= windowprop) ++failed[2];
                }
                break;
            }
        }
        for (int i = cloud->height-1; i >= 0; --i) {
            int idx = j + i*cloud->width;
            if (indices[idx] == index) {
                ++lines[3];
                if (i < cloud->height - windowsize) {
                    int count = 0;
                    for (int k = 1; k < windowsize+1; ++k) {
                        if (isBehind(cloud->at(idx+k*cloud->width), plane)) {
                            ++count;
                        }
                    }
                    if (count >= windowprop) ++failed[3];
                }
                break;
            }
        }
    }
    int numfailed = 0;
    for (int i = 0; i < 4; ++i) {
        if (failed[i]/(double) lines[i] > sideprop) ++numfailed;
    }
    return numfailed;
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

void findPlanesWithNormals(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    if (cloud->height == 1) {
        cerr << "Error: findPlanesWithNormals expects a range image" << endl;
        return;
    }
    // Compute normals
    PointCloud<PointNormal>::Ptr filtered(new PointCloud<PointNormal>);
    copyPointCloud(*cloud, *filtered);
    IntegralImageNormalEstimation<PointXYZ, PointNormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(12.0f);
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
                if (!isValid(cloud->at(x,y)) || isnan(filtered->at(x,y).normal_x)) continue;
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
                            if (v.dot(avgv) < sin(2*ANGLETHRESHOLD)) {
                                nx.push(xx); ny.push(yy);
                                visited[yy*cloud->width+xx] = -2;
                            }
                        }
                    }
                }
            }
            //if (cnt/(double) cloud->size() > MININLIERPROPORTION)
            if (cnt > MININLIERCOUNT) {
                candidates.push_back(Vector4d(avgv(0), avgv(1), avgv(2), 0));
                candidateids.push_back(n);
                sizes.push_back(make_pair(cnt, sizes.size()));
            }
            n++;
        }
    }

    // Recompute average offset
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

    // Relabel points based on successful planes
    for (int i = 0; i < cloud->size(); ++i) {
        for (int j = 0; j < planes.size(); ++j) {
            if (visited[i] == candidateids[j]) {
                ids[i] = j;
                break;
            }
        }
        /*if (ids[i] == -1) {
            for (int j = 0; j < planes.size(); ++j) {
                if (!isValid(cloud->at(i)) || !onPlane(planes[j], cloud->at(i))) continue;
                if (isValid(cloud->at(i)) && isnan(filtered->at(i).normal_x)) {
                    // If null normal, it's good enough to just be on the plane
                    ids[i] = j;
                } else {
                    Vector3d norm = planes[j].head(3);
                    if (getNormal(filtered->at(i)).dot(norm) > cos(2*ANGLETHRESHOLD)) {
                        ids[i] = j;
                    }
                }
            }
        }*/
    }

    // Add labels to neighboring null normals
    vector<int> dists(ids.size(), 0);
    vector<int> nearest(ids.size(), -1);
    for (int i = 0; i < cloud->height; ++i) {
        int last = -1;
        int lastj = -1;
        for (int j = 0; j < cloud->width; ++j) {
            int idx = j + i*cloud->width;
            if (ids[idx] == -1 && last > -1) {
                if (!isValid(cloud->at(idx)) || !onPlane(planes[last], cloud->at(idx)) || !isnan(filtered->at(idx).normal_x)) continue;
                nearest[idx] = last;
                dists[idx] = j - lastj;
            } else {
                last = ids[idx];
                lastj = j;
            }
        }
        last = -1;
        lastj = -1;
        for (int j = cloud->width-1; j>=0; --j) {
            int idx = j + i*cloud->width;
            if (ids[idx] == -1 && last > -1) {
                if (!isValid(cloud->at(idx)) || !onPlane(planes[last], cloud->at(idx)) || !isnan(filtered->at(idx).normal_x)) continue;
                if (dists[idx] > j - lastj) {
                    nearest[idx] = last;
                    dists[idx] = j - lastj;
                }
            } else {
                last = ids[idx];
                lastj = j;
            }
        }
    }
    for (int j = 1; j < cloud->width; ++j) {
        int last = -1;
        int lastj = -1;
        for (int i = 0; i < cloud->height; ++i) {
            int idx = j + i*cloud->width;
            if (ids[idx] == -1 && last > -1) {
                if (!isValid(cloud->at(idx)) || !onPlane(planes[last], cloud->at(idx)) || !isnan(filtered->at(idx).normal_x)) continue;
                nearest[idx] = last;
                dists[idx] = j - lastj;
            } else {
                last = ids[idx];
                lastj = j;
            }
        }
        last = -1;
        lastj = -1;
        for (int i = cloud->height-1; i >= 0; --i) {
            int idx = j + i*cloud->width;
            if (ids[idx] == -1 && last > -1) {
                if (!isValid(cloud->at(idx)) || !onPlane(planes[last], cloud->at(idx)) || !isnan(filtered->at(idx).normal_x)) continue;
                if (dists[idx] > j - lastj) {
                    nearest[idx] = last;
                    dists[idx] = j - lastj;
                }
            } else {
                last = ids[idx];
                lastj = j;
            }
        }
    }
    for (int i = 0; i < ids.size(); ++i) {
        if (nearest[i] > -1) ids[i] = nearest[i];
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
                if (!isValid(cloud->at(x,y))) continue;
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

    Vector3d avgpt = cloudMidpoint(cloud);

    for (int i = 0; i < origplanes.size(); ++i) {
        origplanes[i](3) += avgpt.dot(origplanes[i].head(3));
    }

    int n = 0;
    for (int i = 0; i < origplanes.size(); ++i) {
        if (relabel[i] == -1) {
            relabel[i] = n;
            planes.push_back(origplanes[i]);
            for (int j = i+1; j < origplanes.size(); ++j) {
                if (origplanes[i].head(3).dot(origplanes[j].head(3)) > cos(ANGLETHRESHOLD)) {
                    if (abs(origplanes[i](3) - origplanes[j](3)) < FUSETHRESHOLD) {
                        relabel[j] = n;
                    }
                }
            }
            ++n;
        }
    }
    for (int i = 0; i < origplanes.size(); ++i) {
        origplanes[i](3) -= avgpt.dot(origplanes[i].head(3));
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

void filterAngleVariance(
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids,
        double deviationthreshold = ANGULARDEVIATIONTHRESHOLD)
{
    // Estimate normals
    PointCloud<PointNormal>::Ptr filtered(new PointCloud<PointNormal>);
    copyPointCloud(*cloud, *filtered);
    IntegralImageNormalEstimation<PointXYZ, PointNormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(12.0f);
    ne.setInputCloud(cloud);
    ne.compute(*filtered);
    // Compute average angular deviation from normal
    vector<double> std(planes.size(), 0);
    vector<int> counts(planes.size(), 0);
    for (int i = 0; i < filtered->size(); ++i) {
        if (ids[i] > -1 && !isnan(filtered->at(i).normal_x)) {
            double theta = getNormal(filtered->at(i)).dot(planes[ids[i]].head(3));
            std[ids[i]] += safe_acos(theta);
            ++counts[ids[i]];
        }
    }

    // Filter planes
    vector<Vector4d> origplanes(planes);
    vector<int> newids;
    planes.clear();
    int n = 0;
    for (int i = 0; i < origplanes.size(); ++i) {
        if (std[i]/counts[i] < deviationthreshold) {
            planes.push_back(origplanes[i]);
            newids.push_back(n++);
        }
        else {
            newids.push_back(-1);
        }
    }
    // Relabel points
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] > -1) {
            ids[i] = newids[ids[i]];
        }
    }
}

void filterBoundingSides(
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids)
{
    // Filter planes
    vector<Vector4d> origplanes(planes);
    vector<int> newids;
    planes.clear();
    int n = 0;
    for (int i = 0; i < origplanes.size(); ++i) {
        if (calculateBoundingSides(planes[i], cloud, ids, i) < 3) {
            planes.push_back(origplanes[i]);
            newids.push_back(n++);
        }
        else {
            newids.push_back(-1);
        }
    }
    // Relabel points
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] > -1) {
            ids[i] = newids[ids[i]];
        }
    }
}

void recalculateOffsets(
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids)
{
    for (int i = 0; i < planes.size(); ++i) {
        planes[i](3) = 0;
    }
    // Recompute average offset
    vector<int> sizes(planes.size(), 0);
    for (int i = 0; i < cloud->size(); ++i) {
        for (int j = 0; j < planes.size(); ++j) {
            if (ids[i] == j) {
                ++sizes[ids[i]];
                planes[j](3) -= planes[j](0)*cloud->at(i).x
                              + planes[j](1)*cloud->at(i).y
                              + planes[j](2)*cloud->at(i).z;
            }
        }
    }
    for (int i = 0; i < planes.size(); ++i) {
        planes[i](3) /= sizes[i];
    }
}

void filterSize(
        int minsize,
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids)
{
    vector<int> sizes(planes.size(), 0);
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] > -1) sizes[ids[i]]++;
    }
    vector<int> newids;
    int n = 0;
    vector<Vector4d> origplanes(planes);
    planes.clear();
    for (int i = 0; i < origplanes.size(); ++i) {
        if (sizes[i] > minsize) {
            planes.push_back(origplanes[i]);
            newids.push_back(n++);
        }
        else newids.push_back(-1);
    }
    // Relabel points
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] > -1) {
            ids[i] = newids[ids[i]];
        }
    }
}

void recalculateNormals(
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids)
{
    vector<Matrix<double, 3, Dynamic> > m(planes.size());
    vector<int> counts(planes.size(), 0);
    vector<Vector3d> centroids(planes.size(), Vector3d::Zero());
    for (int i = 0; i < cloud->size(); ++i) {
        if (ids[i] > -1) {
            counts[ids[i]]++;
            centroids[ids[i]] += Vector3d(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
        }
    }
    for (int i = 0; i < planes.size(); ++i) {
        m[i].resize(3,counts[i]);
        centroids[i] /= counts[i];
        counts[i] = 0;
    }
    for (int i = 0; i < cloud->size(); ++i) {
        if (ids[i] > -1) {
            m[ids[i]](0,counts[ids[i]]) = cloud->at(i).x - centroids[ids[i]](0);
            m[ids[i]](1,counts[ids[i]]) = cloud->at(i).y - centroids[ids[i]](1);
            m[ids[i]](2,counts[ids[i]]) = cloud->at(i).z - centroids[ids[i]](2);
            counts[ids[i]]++;
        }
    }
    for (int i = 0; i < planes.size(); ++i) {
        JacobiSVD<Matrix<double, 3, Dynamic> > svd(m[i], ComputeFullU);
        Vector3d v = svd.matrixU().col(2);
        if (v.dot(planes[i].head(3)) < 0) v = -v;
        planes[i].head(3) = v;
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
    recalculateNormals(cloud, planes, ids);
    recalculateOffsets(cloud, planes, ids);
    for (int i = 0; i < 2; ++i) {
        combineLikePlanes(cloud, planes, ids);
        recalculateNormals(cloud, planes, ids);
        recalculateOffsets(cloud, planes, ids);
    }
    filterAngleVariance(cloud, planes, ids);
    if (planes.size() > 1) {
        filterBoundingSides(cloud, planes, ids);
    }
    //filterSize(MINPLANESIZE, cloud, planes, ids);
}

int setMinPlaneInliers(int n) {
    MININLIERCOUNT = n;
    return MININLIERCOUNT;
}
double setAngleThreshold(double d) {
    ANGLETHRESHOLD = d;
    return ANGLETHRESHOLD;
}
