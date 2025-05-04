#ifndef  SNAKE_GAIT_HPP
#define  SNAKE_GAIT_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>




class PedaliGaitModoki{
    public:
        PedaliGaitModoki(int joint_num_, double link_len_): joint_num(joint_num_), link_len(link_len_){
            //動きの様子を見て変更・調整
            wave_num = 2.0;
            serpenoid_alp = 1.7;
            vertical_alp_adjustment = 0.10;
            //初期化
            curve_len_unit = (link_len*joint_num/wave_num)/4.0;
            curve_k = M_PI/(2*curve_len_unit);
            shape_direction = 1;
            target_angle.resize(joint_num, 0.0);
        }

    public:
        std::vector<double> calc_joint(double s_h, double psi){
            for(int i=1; i<=joint_num; i++){
                if(i%2==1){
                    target_angle[i-1] = shape_direction*std::sin(psi)*(serpenoid_alp*vertical_alp_adjustment)*(integral_sin(2*curve_k, s_h, i)+1.0) + std::cos(psi)*serpenoid_alp*integral_cos(curve_k, s_h, i);
                }else{
                    target_angle[i-1] = shape_direction*std::cos(psi)*(serpenoid_alp*vertical_alp_adjustment)*(integral_sin(2*curve_k, s_h, i)+1.0) - std::sin(psi)*serpenoid_alp*integral_cos(curve_k, s_h, i);
                }
            }
            return target_angle;
        }
    
        void invert_shape(){
            shape_direction = shape_direction*(-1);
        }

    private:
        int joint_num;
        double link_len;
        double wave_num;
        double curve_len_unit;
        double curve_k;
        double serpenoid_alp;
        double vertical_alp_adjustment;
        int shape_direction;
        std::vector<double> target_angle;

        double integral_cos(double c_k, double s_h, int i){
            return -std::cos(c_k*(s_h-(i-1)*link_len)) + std::cos(c_k*(s_h-(i+1)*link_len));
        }
        double integral_sin(double c_k, double s_h, int i){
            return std::sin(c_k*(s_h-(i-1)*link_len)) - std::sin(c_k*(s_h-(i+1)*link_len));
        }
};


class LateralGait{
    public:
        LateralGait(int joint_num_, double link_len_): joint_num(joint_num_), link_len(link_len_){
            //動きの様子を見て変更・調整
            kappa = 0.4; //曲率一定=円弧
            //初期化
            shape_direction = 1;
            target_angle.resize(joint_num, 0.0);
        }

    public:
        std::vector<double> calc_joint(double psi){
            for(int i=0; i<joint_num; i++){
                if(i%2==0){
                    target_angle[i] = -std::sin(psi)*kappa*2*link_len * shape_direction;
                }else{
                    target_angle[i] =  std::cos(psi)*kappa*2*link_len * shape_direction;
                }
            }
            return target_angle;
        }

        std::vector<double> calc_joint(double psi, double k){
            kappa = k;
            return calc_joint(psi);
        }

        void invert_shape(){
            shape_direction = shape_direction*(-1);
        }

    private:
        int joint_num;
        double link_len;
        double kappa;
        int shape_direction;
        std::vector<double> target_angle;
};


class HelicalGait{
    public:
        HelicalGait(int joint_num_, double link_len_): joint_num(joint_num_), link_len(link_len_) {
            // 初期パラメータ設定
            helix_r = 0.80; //0.18;
            helix_h = 0.25 / (2.0 * M_PI);
    
            // カッパ・タウの初期計算（helix_rの方）
            kappa = calc_kappa();
            tau   = calc_tau();
            shape_direction = 1;
            target_angle.resize(joint_num, 0.0);
        }
    
    public:
        std::vector<double> calc_joint(double s_h, double psi){
            for (int i = 1; i <= joint_num; ++i) {    
                if (i % 2 == 1) {
                    target_angle[i-1] = kappa / tau * (std::cos(tau * (s_h - (i-1)*link_len) + psi) - std::cos(tau*(s_h - (i+1)*link_len) + psi)) * shape_direction;
                } else {
                    target_angle[i-1] = kappa / tau * (std::sin(tau * (s_h - (i-1)*link_len) + psi) - std::sin(tau*(s_h - (i+1)*link_len) + psi)) * shape_direction;
                }
            }
            return target_angle;
        }

        std::vector<double> calc_joint(double s_h, double psi, double r){
            helix_r = r;
            kappa = calc_kappa();
            tau   = calc_tau();
            return calc_joint(s_h, psi);
        }

        std::vector<double> calc_joint(double s_h, double psi, double r, double h){
            helix_h = h;
            return calc_joint(s_h, psi, r);
        }

        void invert_shape(){
            shape_direction = shape_direction*(-1);
        }
        
    
    private:
        int joint_num;
        double link_len;
    
        double helix_r;
        double helix_h;
        double kappa;
        double tau;
        int shape_direction;
        std::vector<double> target_angle;

        double calc_kappa(){
            return helix_r/(helix_r*helix_r + helix_h*helix_h);
        }
        double calc_tau(){
            return helix_h/(helix_r*helix_r + helix_h*helix_h);
        }
    };


class SnakeGait{
    public:
        SnakeGait(int joint_num_, double link_len_): joint_num(joint_num_), link_len(link_len_), 
                                                     pedal_gait(joint_num_, link_len_), lateral_gait(joint_num_, link_len_), helical_gait(joint_num_, link_len_){
            s_h  = joint_num*(link_len);
            psi = 0.0;
            target_angle.resize(joint_num, 0.0);
            current_angle.resize(joint_num, 0.0); 
            next_target_angle.resize(joint_num, 0.0); 
        }

    private:
        int joint_num;
        double link_len;
        double s_h;
        double psi;
        double radius;
        std::string current_gait;
        std::string next_gait;
        std::vector<double> current_angle;
        std::vector<double> target_angle;
        std::vector<double> next_target_angle;

    public:
        PedaliGaitModoki pedal_gait;
        LateralGait lateral_gait;
        HelicalGait helical_gait;

        void ready_next_gait(std::string next_gait_, std::vector<double> current_angle_, double s_h_, double psi_, double r){
            next_gait = next_gait_;
            current_angle = current_angle_;
            s_h = s_h_;
            psi = psi_;
            radius = r;

            if(next_gait=="Pedal"){
                next_target_angle = pedal_gait.calc_joint(s_h, psi);
            }else if(next_gait=="Lateral"){
                next_target_angle = lateral_gait.calc_joint(psi, 1/radius);
            }else if(next_gait=="Helical"){
                next_target_angle = helical_gait.calc_joint(s_h, psi, radius);
            }
            //std::cout << "next_target_angle.size(): " << next_target_angle.size() << std::endl;
        }
        void invert_gait(std::string gait, std::vector<double> current_angle_, double s_h_, double psi_, double r){
            next_gait = gait;
            current_angle = current_angle_;
            s_h = s_h_;
            psi = psi_;
            radius = r;

            pedal_gait.invert_shape();
            lateral_gait.invert_shape();
            helical_gait.invert_shape();

            if(gait=="Pedal"){
                next_target_angle = pedal_gait.calc_joint(s_h, psi);
            }else if(gait=="Lateral"){
                next_target_angle = lateral_gait.calc_joint(psi, 1/radius);
            }else if(gait=="Helical"){
                next_target_angle = helical_gait.calc_joint(s_h, psi, radius);
            }else{
                std::fill(next_target_angle.begin(), next_target_angle.end(), 0.0);
            }
            //std::cout << "next_target_angle.size(): " << next_target_angle.size() << std::endl;
        }

        void update_current_gait(){
            current_gait = next_gait;
        }

        //lateral<->helicalでgait変更
        std::vector<double> switch_gait(double ratio){
            ratio = std::max(std::min(ratio, 1.0), 0.0);
            //std::cout << next_target_angle.size() << ", " << current_angle.size() << std::endl;
            for(int i=0; i<joint_num; i++){
                target_angle[i] = ratio*next_target_angle[i] + (1-ratio)*current_angle[i];
            }
            return target_angle;
        }

        // pedalでinvert、pedal<->lateral, helicalでgait変更
        std::vector<double> switch_gait_via0(double ratio){
            ratio = std::max(std::min(ratio, 1.0), 0.0);
            //std::cout << next_target_angle.size() << ", " << current_angle.size() << std::endl;
            if(ratio <= 0.5){
                for(int i=0; i<joint_num; i++){
                    target_angle[i] = (1-2*ratio)*current_angle[i];
                }
            }else{
                for(int i=0; i<joint_num; i++){
                    target_angle[i] = 2*(ratio-0.5)*next_target_angle[i];
                }
            }
            return target_angle;
        }



        // lateral, helicalでinvertするとき
        std::vector<double> invert_gait_lateral_or_helix(double ratio){ //逆の形状にするときに前半部を最初に曲率をk→0→-kして、その後に後半部を
            
            if(ratio<=0){
                if(current_gait=="Lateral"){
                    target_angle = lateral_gait.calc_joint(psi, 1/(-radius));
                }else if(current_gait=="Helical"){
                    target_angle = helical_gait.calc_joint(s_h, psi, -radius);
                }
                current_angle = target_angle;
                return target_angle;
            }else if(ratio>=1){
                if(current_gait=="Lateral"){
                    target_angle = lateral_gait.calc_joint(psi, 1/radius);
                }else if(current_gait=="Helical"){
                    target_angle = helical_gait.calc_joint(s_h, psi, radius);
                }
                current_angle = target_angle;
                return target_angle;
            }

            ratio = std::max(std::min(ratio, 1.0), 0.0);
            //std::cout << next_target_angle.size() << ", " << current_angle.size() << std::endl;
            target_angle = current_angle;

            double r;
            std::vector<double> target_angle_tmp;

            if(ratio <= 0.50){
                if(ratio <= 0.25){
                    r = -((1-4*ratio)*radius + 4*ratio*5.0);
                }else{ // 0.25 < ratio < 0.50
                    r = 4*(ratio-0.25)*radius + (1-4*(ratio-0.25))*5.0;
                }

                if(current_gait=="Lateral"){
                    target_angle_tmp = lateral_gait.calc_joint(psi, 1/r);
                }else if(current_gait=="Helical"){
                    target_angle_tmp = helical_gait.calc_joint(s_h, psi, r);
                }

                for(int i=0; i<int(joint_num/2); i++){
                    target_angle[i] = target_angle_tmp[i];
                }
            }else{
                if(ratio <= 0.75){
                    r = -((1-4*(ratio-0.5))*radius + 4*(ratio-0.5)*5.0);
                }else{ // 0.75 < ratio < 1.0
                    r = 4*(ratio-0.75)*radius + (1-4*(ratio-0.75))*5.0;
                }

                if(current_gait=="Lateral"){
                    target_angle_tmp = lateral_gait.calc_joint(psi, 1/r);
                }else if(current_gait=="Helical"){
                    target_angle_tmp = helical_gait.calc_joint(s_h, psi, r);
                }

                for(int i=joint_num-int(joint_num/2)-joint_num%2; i<joint_num; i++){
                    target_angle[i] = target_angle_tmp[i];
                }
            }
            //std::cout << "r: " << r <<std::endl;
            current_angle = target_angle;
            return target_angle;
        }

        std::vector<double> switch_or_invert_gait(double ratio){
            if(current_gait=="Pedal" || next_gait=="Pedal"){ // pedal<->pedalでinvert、またはpedal<->lateral, helicalでgait変更
                switch_gait_via0(ratio);
            }else if( (current_gait=="Lateral" && next_gait=="Helical") || (current_gait=="Helical" && next_gait=="Lateral") ){ //lateral<->helicalでgait変更
                switch_gait(ratio);
            }else if ( (current_gait=="Lateral" && next_gait=="Lateral") || (current_gait=="Helical" && next_gait=="Helical")){ //lateral, helicalでinvert
                invert_gait_lateral_or_helix(ratio);
            }else{
                switch_gait(ratio);
            }

            if(ratio>=1.0){
                update_current_gait();
            }
            return target_angle;
        }

};

#endif //SNAKE_GAIT_HPP
