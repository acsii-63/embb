#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <ros/ros.h>


class FeatureExtract {
typedef std::pair<Eigen::Vector3d,Eigen::Vector3d> PairVector;
#define MathPI 3.14159
private:
    std::vector<Eigen::Vector3d> Pos;
    std::vector<Eigen::Vector3d> Vel;
    std::vector<Eigen::Vector3d> Acc;
    std::vector<double> Yaw;
    std::vector<double> YawVel;
    std::vector<int> Index;

    std::vector<double> time; 

    Eigen::Vector3d SamplePos;
    Eigen::Vector3d SampleVel;
    Eigen::Vector3d SampleAcc;
    Eigen::Vector3d SampleJer;
    Eigen::Vector3d SampleSna;

    double SampleYawPos;
    
    std::vector<PairVector> SetPoint;
    int setpointNum = 0;

    const Eigen::Vector3d VZero = Eigen::Vector3d::Zero();
    const int instance = 800;
    const double dt = 0.01;
    const double max_velocity = 2.5;
    const double max_snap = 1.25;
    const double max_pos = 10.0; 

    const double max_yawsnap = MathPI*0.025;    

    enum TrackStatus  {FullTrack,
                       CuttedTrack,
                       Failed};

public:
    // Default constructor
    FeatureExtract(){};
    void setStart(Eigen::Vector3d start_pos ,double start_yaw ){
        SampleYawPos = start_yaw;
        SamplePos = start_pos;
        SampleVel = VZero;
        SampleAcc = VZero;
        SampleJer = VZero;
        SampleSna = VZero;
        Pos.push_back(SamplePos);
        Vel.push_back(SampleVel);
        Acc.push_back(SampleAcc);
        Yaw.push_back(SampleYawPos);
    };

    int getSize(){
        return Pos.size();
    }
    void getWpIndex(std::vector<int>& Id ){
        Id = Index;
    }
    Eigen::Vector3d getPos(int idx){
        return Pos[idx];
    }
    Eigen::Vector3d getVel(int idx){
        return Vel[idx];
    }
    Eigen::Vector3d getAcc(int idx){
        return Acc[idx];
    }
    double getYaw(int idx){
        return Yaw[idx];
    }
  
    void appendSetpoint(PairVector pair) {
        SetPoint.push_back(pair);
        setpointNum++;
    }
    void solveYaw(Eigen::Vector3d view){
        double YawDiff = YawDistance(SampleYawPos, view - SamplePos);
        while (fabs(YawDiff)>0.1){
            double YawUpdate = YawDiff * std::min(max_yawsnap / fabs(YawDiff),1.0);
            SampleYawPos  += YawUpdate;
            YawDiff -= YawUpdate;
            Pos.push_back(SamplePos);
            Vel.push_back(SampleVel);
            Acc.push_back(SampleAcc);
            Yaw.push_back(SampleYawPos);
        }
    }
    void updateIndex(int i){
        Index.push_back(i);
    }
    void solveSp(const Eigen::Vector3d Distance , const Eigen::Vector3d& view){
        
        Eigen::Vector3d NormDistance = Distance.normalized();
        Eigen::Vector3d RelScale;
        int PrimIdx;
        double PrimScale;
        findScale(NormDistance,RelScale,PrimIdx,PrimScale);

        double FreeTime = 0, Snap =0 , SnapYaw2 ;
        TrackStatus track = calculate(Distance(PrimIdx),FreeTime,PrimScale,Snap);
        std::vector<int> TriagleTrans = triagle(FreeTime);
        for (int i=0; i<TriagleTrans.size(); i++) {

            SampleSna = TriagleTrans[i] * RelScale * Snap;
            SampleJer += SampleSna * dt;
            SampleAcc += SampleJer * dt;
            SampleVel += SampleAcc * dt;
            SamplePos += SampleVel * dt;
            
            double YawDiff = YawDistance(SampleYawPos, view - SamplePos);
            SampleYawPos  += YawDiff;
            
            Pos.push_back(SamplePos);
            Vel.push_back(SampleVel);
            Acc.push_back(SampleAcc);
            Yaw.push_back(SampleYawPos);
        }
    }
    void findScale(Eigen::Vector3d& dis , Eigen::Vector3d& relScale , int& main , double& primScale){
        main = MaxIndex(dis);
        relScale = dis/dis(main);
        primScale = dis(main);
    }

    TrackStatus calculate(double distance , double& freeTime ,const double &scale , double& snap){
        if(fabs(distance) > max_pos * fabs(scale)){
            freeTime = (distance - max_pos * scale) / (max_velocity * scale); 
            snap = max_snap * scale;
            return FullTrack;
        }
        else {
            freeTime = 0;
            snap = max_snap * distance / max_pos;
            return CuttedTrack;
        }
    }

    std::vector<int> triagle(double freeTime){
        std::vector<int> up(instance/8, 1); 
        std::vector<int> down(instance/8,-1);       
        int freeInstance = freeTime / dt;
        std::vector<int> fre(freeInstance, 0);
        std::vector<int> tri;
        tri.insert(tri.end(), up.begin(), up.end());
        tri.insert(tri.end(), down.begin(), down.end());
        tri.insert(tri.end(), down.begin(), down.end());
        tri.insert(tri.end(), up.begin(), up.end());
        if(freeInstance > 0)
        tri.insert(tri.end(), fre.begin(), fre.end());
        tri.insert(tri.end(), down.begin(), down.end());
        tri.insert(tri.end(), up.begin(), up.end());
        tri.insert(tri.end(), up.begin(), up.end());
        tri.insert(tri.end(), down.begin(), down.end());
        return tri;
    }
    int MaxIndex(const Eigen::Vector3d& vec) {
        int maxIndex = 0;
        double maxAbs = std::abs(vec(0));
        for (int i = 1; i < 3; ++i) {
            if (std::abs(vec(i)) > maxAbs) {
                maxIndex = i;
                maxAbs = std::abs(vec(i));
            }
        }
    return maxIndex;
    }
    double YawDistance(double yaw1 , Eigen::Vector3d d2){
        double yaw2 = std::atan2( d2.y() , d2.x());
        double yawdiff = yaw2 - yaw1;
            while(fabs(yawdiff)> fabs(yawdiff-2*MathPI)){
              yawdiff-=2*MathPI;
            }
            while(fabs(yawdiff)> fabs(yawdiff+2*MathPI)){
              yawdiff+=2*MathPI;
            }
        return yawdiff;
    }
    void reZero(){
        SampleVel = VZero;
        SampleAcc = VZero;
        SampleJer = VZero;
        SampleSna = VZero;
    }
    void execute(){
        for (int i = 0; i <SetPoint.size(); i++){
            reZero();
            solveYaw(SetPoint[i].second);
            Eigen::Vector3d distance = SetPoint[i].first - SamplePos;
            solveSp(distance,SetPoint[i].second);
            updateIndex(Pos.size());
        }
        updateIndex(Pos.size());
    }
};
