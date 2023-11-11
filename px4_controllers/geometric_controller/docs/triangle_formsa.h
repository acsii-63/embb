#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <ros/ros.h>

class TriangleForm {
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
    double SampleYawVel;
    double SampleYawAcc;
    double SampleYawJer;
    double SampleYawSna;
    
    std::vector<Eigen::Vector3d> SetPoint;
    int setpointNum = 0;

    const Eigen::Vector3d VZero = Eigen::Vector3d::Zero();
    const int instance = 800;
    const double dt = 0.01;
    const double max_velocity = 2.5;
    const double max_snap = 1.25;
    const double max_pos = 10.0; 

    const double max_yawsnap = 0.5;
    const double max_yawpos = 4;     

    enum TrackStatus  {FullTrack,
                       CuttedTrack,
                       Failed};

public:
    // Default constructor
    TriangleForm(){};
    void setStart(Eigen::Vector3d start_pos ,double start_yaw ){
        SampleYawPos = start_yaw;
        SampleYawVel = 0;
        SampleYawAcc = 0;
        SampleYawJer = 0;
        SampleYawSna = 0;

        SamplePos = start_pos;
        SampleVel = VZero;
        SampleAcc = VZero;
        SampleJer = VZero;
        SampleSna = VZero;
        Pos.push_back(SamplePos);
        Vel.push_back(SampleVel);
        Acc.push_back(SampleAcc);
        Yaw.push_back(SampleYawPos);
        YawVel.push_back(SampleYawVel);
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
    double getYawVel(int idx){
        return YawVel[idx];
    }
  
    void appendSetpoint(Eigen::Vector3d& sp) {
        SetPoint.push_back(sp);
        setpointNum++;
    }
    void solveYaw(double YawDistance , double& SnapYaw1){
        double SnapYaw2 ;
        TrackStatus trackYaw = calculateYaw(YawDistance,SnapYaw2);
        std::vector<double> TriagleYaw = triagle_yaw(SnapYaw2);
        for (auto ty : TriagleYaw) {
            SampleYawSna  = ty;
            SampleYawJer  += SampleYawSna * dt;
            SampleYawAcc  += SampleYawJer * dt;
            SampleYawVel  += SampleYawAcc * dt;
            SampleYawPos  += SampleYawVel * dt;
            
            Pos.push_back(SamplePos);
            Vel.push_back(SampleVel);
            Acc.push_back(SampleAcc);
            Yaw.push_back(SampleYawPos);
            YawVel.push_back(SampleYawVel);
        }
        SnapYaw1 = SnapYaw2;
    }
    void updateIndex(int i){
        Index.push_back(i);
    }
    void solveSp(const Eigen::Vector3d Distance , double YawDistance , double& SnapYaw1){
        
        Eigen::Vector3d NormDistance = Distance.normalized();
        Eigen::Vector3d RelScale;
        int PrimIdx;
        double PrimScale;
        findScale(NormDistance,RelScale,PrimIdx,PrimScale);

        double FreeTime = 0, Snap =0 , SnapYaw2 ;
        TrackStatus track = calculate(Distance(PrimIdx),FreeTime,PrimScale,Snap);
        TrackStatus trackYaw = calculateYaw(YawDistance,SnapYaw2);
        std::vector<int> TriagleTrans = triagle(FreeTime);
        std::vector<double> TriagleYaw = triagle_yaw(FreeTime,SnapYaw1,SnapYaw2);
        for (int i=0; i<TriagleTrans.size(); i++) {

            SampleSna = TriagleTrans[i] * RelScale * Snap;
            SampleJer += SampleSna * dt;
            SampleAcc += SampleJer * dt;
            SampleVel += SampleAcc * dt;
            SamplePos += SampleVel * dt;
            
            SampleYawSna  = TriagleYaw[i];
            SampleYawJer  += SampleYawSna * dt;
            SampleYawAcc  += SampleYawJer * dt;
            SampleYawVel  += SampleYawAcc * dt;
            SampleYawPos  += SampleYawVel * dt;
            
            Pos.push_back(SamplePos);
            Vel.push_back(SampleVel);
            Acc.push_back(SampleAcc);
            Yaw.push_back(SampleYawPos);
            YawVel.push_back(SampleYawVel);
        }
        SnapYaw1 = SnapYaw2;
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
    TrackStatus calculateYaw(double distance , double& snap){
            snap = max_yawsnap * distance / max_yawpos;
            return CuttedTrack;
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
    std::vector<double> triagle_yaw(double snap2){
        std::vector<double> up2(instance/8, snap2); 
        std::vector<double> down2(instance/8,-snap2);       
        std::vector<double> tri;
        tri.insert(tri.end(), up2.begin(), up2.end());
        tri.insert(tri.end(), down2.begin(), down2.end());
        tri.insert(tri.end(), down2.begin(), down2.end());
        tri.insert(tri.end(), up2.begin(), up2.end());
        return tri;
    }
    std::vector<double> triagle_yaw(double freeTime,double snap1,double snap2){
        std::vector<double> up1(instance/8, snap1); 
        std::vector<double> down1(instance/8,-snap1);
        std::vector<double> up2(instance/8, snap2); 
        std::vector<double> down2(instance/8,-snap2);       
        int freeInstance = freeTime / dt;
        std::vector<double> fre(freeInstance, 0);
        std::vector<double> tri;

        tri.insert(tri.end(), down1.begin(), down1.end());
        tri.insert(tri.end(), up1.begin(), up1.end());
        tri.insert(tri.end(), up1.begin(), up1.end());
        tri.insert(tri.end(), down1.begin(), down1.end());

        if(freeInstance > 0)
        tri.insert(tri.end(), fre.begin(), fre.end());

        tri.insert(tri.end(), up2.begin(), up2.end());
        tri.insert(tri.end(), down2.begin(), down2.end());
        tri.insert(tri.end(), down2.begin(), down2.end());
        tri.insert(tri.end(), up2.begin(), up2.end());
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
    double YawDistance(Eigen::Vector3d d1 , Eigen::Vector3d d2){
        double yaw1 = std::atan2( d1.y() , d1.x());
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
        SampleYawAcc = 0;
        SampleYawJer = 0;
        SampleYawSna = 0;
    }
    void execute(){
        double snapyaw =0,yaw_distance = 0;
        solveYaw(YawDistance(SampleYawPos,SetPoint[0]-SamplePos),snapyaw);
        for (int i = 0; i <SetPoint.size(); i++){
        reZero();
        updateIndex(Pos.size());
        if(i < SetPoint.size()-1){
        Eigen::Vector3d distance = SetPoint[i] - SamplePos;
        Eigen::Vector3d distance2 = SetPoint[i+1] - SetPoint[i];
        solveSp(distance,YawDistance(distance,distance2),snapyaw);
        }
        else{
        Eigen::Vector3d distance = SetPoint[i] - SamplePos;    
        solveSp(distance,0,snapyaw);
        } 
        }
        updateIndex(Pos.size());
    }
};
