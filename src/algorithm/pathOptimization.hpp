#pragma once
#include <vector>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"

class PathOptimization {

private: 
    vector<Point> smoothed;

    double turnCost(const Point& prev, const Point& current, const Point& next) {
        // Calculate direction vectors
        double dx1 = current.x - prev.x;
        double dy1 = current.y - prev.y;
        double dx2 = next.x - current.x;
        double dy2 = next.y - current.y;
        
        // Calculate lengths
        double len1 = sqrt(dx1*dx1 + dy1*dy1);
        double len2 = sqrt(dx2*dx2 + dy2*dy2);
        
        if (len1 < 1e-9 || len2 < 1e-9) return 0;
        
        // Normalize
        dx1 /= len1; dy1 /= len1;
        dx2 /= len2; dy2 /= len2;
        
        // Dot product: 1 = straight, 0 = 90°, -1 = 180°
        double dot = dx1*dx2 + dy1*dy2;
        
        // Turn cost: 0 for straight, higher for sharper turns
        double angle = acos(max(-1.0, min(1.0, dot)));
        return angle;  // Radians: 0 = straight, π/2 = right angle, π = U-turn
    }

    void optimizePath(const Grid& grid, const vector<Point>& path) {
        if (path.size() <= 2) {
            smoothed = path;
            return;
        }
        
        smoothed = path;
        bool changed = true;
        
        while (changed) {
            changed = false;
            
            int currentIdx = 0;
            
            while (currentIdx < (int)smoothed.size() - 1) {
                int bestNext = currentIdx + 1;
                double bestScore = numeric_limits<double>::infinity();
                
                // Try to connect to EVERY point ahead (like greedy shortcut)
                for (int i = (int)smoothed.size() - 1; i > currentIdx; i--) {
                    if (isEdgeValid(grid, smoothed[currentIdx], smoothed[i])) {
                        // Calculate turn cost if we skip to this point
                        double score = 0;
                        
                        // Turn cost at currentIdx (from previous segment)
                        if (currentIdx >= 1) {
                            score += turnCost(smoothed[currentIdx-1], 
                                            smoothed[currentIdx], 
                                            smoothed[i]);
                        }
                        
                        // Turn cost at the target (to next segment)
                        if (i + 1 < smoothed.size()) {
                            score += turnCost(smoothed[currentIdx], 
                                            smoothed[i], 
                                            smoothed[i+1]);
                        }
                        
                        // Also factor in distance (shorter is better)
                        score += distance(smoothed[currentIdx], smoothed[i]) * 0.001;
                        
                        if (score < bestScore) {
                            bestScore = score;
                            bestNext = i;
                        }
                        
                        break;  // Found farthest — but could check all for best turn
                    }
                }
                
                if (bestNext != currentIdx + 1) {
                    // Remove all points between currentIdx and bestNext
                    smoothed.erase(smoothed.begin() + currentIdx + 1, 
                                smoothed.begin() + bestNext);
                    changed = true;
                }
                
                currentIdx++;
            }
        }
    }

public:
    PathOptimization(const Grid& grid, const vector<Point>& path) {
        optimizePath(grid, path);
    }

    const vector<Point>& getPath() const {
        return smoothed;
    }
};