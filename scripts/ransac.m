function [params, numinliers] = ransac(computeParamsFn, data, numIters, subsetSize, inlierFn)
    bestinliers = 0;
    n = size(data,1);
    for i=1:numIters
        subset = data(randperm(n,subsetSize), :);
        p = computeParamsFn(subset);
        currinliers = inlierFn(p,data);
        if length(currinliers) > length(bestinliers)
            bestinliers = currinliers;
        end
    end
    params = computeParamsFn(data(bestinliers,:));
    numinliers = length(bestinliers);
