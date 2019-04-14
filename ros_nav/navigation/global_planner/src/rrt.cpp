/****************
2019.4.11
The homework of robots.
add rrt(s) to the global_planner using the API of ROS
H.Z & MK.J
*****************/

#include <global_planner/rrt.h>
#include <stdlib.h>
#include<costmap_2d/cost_values.h>
#include <math.h>
#include <iostream>
#include<vector>
#include<cstdlib>
#include<ctime>

namespace global_planner {

RRTExpansion::RRTExpansion(PotentialCalculator* p_calc, int xs, int ys) :
    Expander(p_calc, xs, ys),T(ns_),ET(ns_) {
    step_size = nx_/100;
    arrive_offset = 10;
    guide_radius = 50;
    guide_rate = 0.1;
    explore_radius = 14;
}
//Expander need to be rewrite

bool RRTExpansion::calculatePotentials(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                        int num_of_ptcs, float* potential,bool use_connect = false, bool use_goal_guide = true, 
                                        bool use_cut_bridge = false, bool use_rrt_star = false) {
// for obs inflation
    // int i_costs;
    // for(int i = 0; i < ns_ - 2* nx_ - ny_; i++ ){
    //     ROS_WARN("%d",costs_[i]);
    //     if(costs_[i] >= lethal_cost_){

    //         // costs_[i + 1] = (lethal_cost_  -1) ? (costs_[i + 1] < lethal_cost_): lethal_cost_;
    //         // costs_[i - 1] = (lethal_cost_  -1) ? (costs_[i - 1] < lethal_cost_): lethal_cost_;
    //         // costs_[i + 2] = (lethal_cost_  -1 )? (costs_[i + 2] < lethal_cost_): lethal_cost_;
    //         // costs_[i - 2] = (lethal_cost_  -1 )? (costs_[i - 2] < lethal_cost_): lethal_cost_;
    //         // costs_[i + nx_] = (lethal_cost_  -1) ? (costs_[i + nx_]  < lethal_cost_): lethal_cost_;
    //         // costs_[i - nx_] = (lethal_cost_  -1) ? (costs_[i - nx_]  < lethal_cost_): lethal_cost_;
    //         // costs_[i + 2*nx_] = (lethal_cost_  -1) ? (costs_[i + 2*nx_] < lethal_cost_): lethal_cost_;
    //         // costs_[i - 2*nx_] = (lethal_cost_  -1) ? (costs_[i - 2*nx_]  < lethal_cost_): lethal_cost_;
    //         // costs_[i + nx_ + 1] = (lethal_cost_  -1) ? (costs_[i + nx_ + 1] < lethal_cost_): lethal_cost_;
    //         // costs_[i + nx_ - 1] = (lethal_cost_  -1) ? (costs_[i + nx_ - 1] < lethal_cost_): lethal_cost_;
    //         // costs_[i - nx_ + 1] = (lethal_cost_  -1) ? (costs_[i - nx_ + 1]< lethal_cost_): lethal_cost_;
    //         // costs_[i - nx_ - 1] = (lethal_cost_  -1 )? (costs_[i - nx_ - 1] < lethal_cost_): lethal_cost_;
    //         // costs_[i + nx_ + 2] = (lethal_cost_  -1 )? (costs_[i + nx_ + 2]  < lethal_cost_): lethal_cost_;
    //         // costs_[i + nx_ - 2] = (lethal_cost_  -1 )? (costs_[i + nx_ - 2] < lethal_cost_): lethal_cost_;
    //         // costs_[i - nx_ + 2] = (lethal_cost_  -1 )? (costs_[i - nx_ + 2] < lethal_cost_): lethal_cost_;
    //         // costs_[i - nx_ - 2] = (lethal_cost_  -1) ? (costs_[i - nx_ - 2] < lethal_cost_): lethal_cost_;
    //     }
    // }
    ROS_INFO("%d,%d,%d",nx_,ny_,ns_);
    m_use_goal_guide = use_goal_guide;
    if(use_connect){
        if (!connect_rrt(costs_,start_x1,start_y1,end_x1,end_y1,num_of_ptcs,potential)){
            return false;
        }
    }else if(use_rrt_star){
        ROS_WARN("RRT*");
        if(!single_rrt_star(costs_,start_x1,start_y1,end_x1,end_y1,num_of_ptcs,potential)){
            return false;
        }
        ROS_WARN("DONE RRT*");
    }else{
        if(!single_rrt(costs_,start_x1,start_y1,end_x1,end_y1,num_of_ptcs,potential)){
            return false;
        }
    }
    if(use_cut_bridge){
        cut_bridge();
    }

    return true;
}

bool RRTExpansion::calculatePlan(std::vector<std::pair<float, float> >& path){
    Node *pre = &T.Nodes_[T.length - 1];
    Node *cur = &T.Nodes_[T.length - 1];
    std::pair<float, float> current;
    current.first = end_x;
    current.second = end_y;
    path.push_back(current);
    int c = 0;
    while (cur->index_ != toIndex(start_x, start_y)) {
        cur = &T.Nodes_[cur->father_];
        current.first = cur->x_;
        current.second = cur->y_;
        path.push_back(current);
        c++;
    }
        // current.first = cur->x_;
        // current.second = cur->y_;
        // path.push_back(current);
    if(c++>ns_*4){
        return false;
    }
    return true;
}

bool RRTExpansion::improvePlan(std::vector<std::pair<float, float> >& path){
    std::vector<std::pair<float, float> >::iterator cur = path.end()-1;
    std::vector<std::pair<float, float> >::iterator pre = path.begin();
    std::vector<std::pair<float, float> >::iterator itev = path.begin();
    int c = 0;
    for(itev;itev != cur;itev ++){
        ROS_WARN("%f, %f\n",(*itev).first, (*itev).second);
    }
    int k=0;
    while(true){
            // ROS_ERROR("%d",bool(pre == path.end()-1)); nx_, ny_, ns_
            // ROS_INFO("path.end()-1=%f, %f\n",(*(path.end()-1)).first, (*(path.end()-1)).second);
            // ROS_INFO("pre=%f, %f\n",(*pre).first, (*pre).second);
        for(cur; cur != pre+2; cur --){
            // ROS_WARN("2");
            if(is_not_col((*pre).first, (*pre).second, (*cur).first, (*cur).second)){
                path.erase(pre+1, cur);
                pre = cur;
                cur = path.end()-1;
                ROS_INFO("%d:cur=%f, %f\n",k,(*cur).first, (*cur).second);
                ROS_INFO("%d:pre=%f, %f\n",k,(*pre).first, (*pre).second);
                break;
            }
        }
        if( (*(path.end() - 1)).first == (*pre).first && (*(path.end()-1)).second == (*pre).second ){
            break;
        }
        if( (*(path.end() - 2)).first == (*pre).first && (*(path.end()-2)).second == (*pre).second ){
            break;
        }
    }

    for(itev = path.begin();itev != cur;itev ++){
        ROS_WARN("%f, %f\n",(*itev).first, (*itev).second);
    }
    if(c++>ns_*4){
        return false;
    }
    return true;
}

int RRTExpansion::MyRand(int low, int high){

    int a = rand() % (high - low);
    a += low;
    return a;
}
bool RRTExpansion::is_legal(int ind){
    if(ind<2||ind>ns_-2){
        return false;
    }
    //&& !(unknown_ && costs[ind]==costmap_2d::NO_INFORMATION)
    if(costs[ind]>=(lethal_cost_ -2))
    {
        return false;
    }else{
        return true;
    }
}
bool RRTExpansion::is_not_col(int startx, int starty, int endx, int endy){
    int step = 1;
    int  cx = startx;
    int cy = starty;
    int i = 0;
    double xd = 1.0;
    double yd = 1.0;
    if(endx < startx) xd = -1.0;
    if(endy < starty) yd = -1.0;
    double dx = fabs(endx - startx);
    double dy = fabs(endy - starty);
    double theta = atan(dy/dx);
    double length = sqrt(dx * dx + dy * dy);
    int cycle = int(length / step);
        for(int i = 0; i <= cycle; i++ ){
                    // int new_x = int (startx + xd * step * i *cos(theta));
                    // int new_y = int (startx + xd * step * i *sin(theta));
            int new_x = int (startx + xd * (step * i / length) * dx);
            int new_y = int (starty + yd * (step * i / length) * dy);
            if(!is_legal(toIndex(new_x,new_y))){
                return false;
            }
        }

    return true;
}


int RRTExpansion::toward(int startx, int starty, int endx, int endy){
    if(sqrt(1.0 * abs(startx - endx)* abs(startx - endx) + 1.0 * abs(starty - endy)*abs(starty - endy)) < step_size){
        return toIndex(endx, endy);}
    double xd = 1.0;
    double yd = 1.0;
    if(endx < startx) xd = -1.0;
    if(endy < starty) yd = -1.0;
    double dx = abs(endx - startx);
    double dy = abs(endy - starty);
    double len = sqrt(dx*dx+dy*dy);
    int new_x = int (startx + xd * 1.0 * (1.0*step_size / len) * dx );
    int new_y = int (starty + yd * 1.0 * (1.0* step_size / len) * dy );
    return toIndex(new_x,new_y);
}
void RRTExpansion::Reclear(){
    T.Clear();
    ET.Clear();
}
bool RRTExpansion::connect_rrt(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                        int num_of_ptcs, float* potential) 
{

    T.Clear();
    ET.Clear();

    std::fill(potential, potential + ns_, POT_HIGH);
    srand((unsigned)time(NULL));   
    costs = costs_;
    start_x = start_x1;
    start_y = start_y1;
    end_x = end_x1;
    end_y = end_y1;
    int arr_pot = 10;

    int start_i = toIndex(start_x, start_y);
    int goal_i = toIndex(end_x, end_y);
    potential[start_i] = arr_pot;
    potential[goal_i] = arr_pot;

    T.AddNode(start_x, start_y, start_i, start_i);       //father is itself
    ET.AddNode(end_x,end_y,goal_i,goal_i);

    int st_node;
    int et_node;
    int cycle = 0;
    ROS_WARN("into while");
    int flag = 0;
    while (true) {

        if(flag == 1){
            break;
            //end;
        }
        int px = MyRand(0, nx_);
        int py = MyRand(0, ny_);
        if(m_use_goal_guide){
            double random = rand() % 1000 / (float)1000;
            if(random < guide_rate){
                int dx = ((end_x-guide_radius)<=0) ? (10) : (end_x-guide_radius);
                int ux = ((guide_radius+end_x)>=nx_) ? (nx_-10) : (guide_radius+end_x);
                int dy = ((end_y-guide_radius)<=0) ? (10) : (end_y-guide_radius);
                int uy = ((end_y+guide_radius)>=ny_) ? (ny_ - 10) : (end_y+guide_radius);
                px = MyRand(dx, ux);
                py = MyRand(dy, uy);
            }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
            }
        }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
        }
        // find_nearest node
        int m_index = T.Find_Near(px, py);
        int cx = T.Nodes_[m_index].x_;
        int cy = T.Nodes_[m_index].y_;
        if(is_legal(toIndex(px, py))){  // point legal
            if(is_not_col(cx, cy, px, py)){  // no obstacle between them
                int new_index = toward(cx, cy, px, py); // toward one step;
                int ccx = new_index % nx_, ccy = new_index / nx_;
                if(!is_legal(new_index))    continue;
           //     if(!T.IfExist(new_index)){
                    T.AddNode(ccx, ccy, new_index, m_index);
                    potential[new_index] = arr_pot;
           //     }
                //判断结束
                if(T.length >= T.max_length /10){
                return false;
                }
                for(int i = 0; i < ET.length; i++){
                    int ex = ET.Nodes_[i].x_;
                    int ey = ET.Nodes_[i].y_;
                    int dx = abs(ccx - ex);
                    int dy = abs(ccy - ey);
                    if(dx < arrive_offset && dy < arrive_offset && is_not_col(ccx,ccy,ex,ey)){
                        st_node = T.length - 1;
                        et_node = i;
                        flag = 1;
                        break;            // arrive
                    }
                }
            }
        }

        if(m_use_goal_guide){
            double random = rand() % 1000 / (float)1000;
            if(random < guide_rate){
                int dx = ((start_x-guide_radius)<=0) ? (10) : (start_x-guide_radius);
                int ux = ((guide_radius+start_x)>=nx_) ? (nx_-10) : (guide_radius+start_x);
                int dy = ((start_y-guide_radius)<=0) ? (10) : (start_y-guide_radius);
                int uy = ((start_y+guide_radius)>=ny_) ? (ny_ - 10) : (start_y+guide_radius);
                px = MyRand(dx, ux);
                py = MyRand(dy, uy);
            }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
            }
        }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
        }

        // find_nearest node
        m_index = ET.Find_Near(px, py);
        cx = ET.Nodes_[m_index].x_;
        cy = ET.Nodes_[m_index].y_;
        if(is_legal(toIndex(px, py))){  // point legal
            if(is_not_col(cx, cy, px, py)){  // no obstacle between them
                int new_index = toward(cx, cy, px, py); // toward one step;
                int ccx = new_index % nx_, ccy = new_index / nx_;
                if(!is_legal(new_index))    continue;
            //    if(!ET.IfExist(new_index)){
                    ET.AddNode(ccx, ccy, new_index, m_index);
                    potential[new_index] = arr_pot;
              //  }
                    if(ET.length >= ET.max_length/10){
                        return false;
                    }
                //判断结束
                for(int i = 0; i < T.length; i++){
                    int ex = T.Nodes_[i].x_;
                    int ey = T.Nodes_[i].y_;
                    int dx = abs(ccx - ex);
                    int dy = abs(ccy - ey);
                    if(dx < arrive_offset && dy < arrive_offset && is_not_col(ccx,ccy,ex,ey)){
                        st_node = i;
                        et_node = ET.length - 1;
                        flag = 1;
                        break;            // arrive
                    }
                }
            }
        }
    }
    Node* cur = &ET.Nodes_[et_node];
    T.AddNode(cur->x_,cur->y_,cur->index_,st_node);
    while(cur ->index_ != goal_i){
        cur = &ET.Nodes_[cur->father_];
        T.AddNode(cur->x_,cur->y_,cur->index_,T.length - 1);
    }

    return true;
}
bool RRTExpansion::single_rrt(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                        int num_of_ptcs, float* potential) 
{
    std::fill(potential, potential + ns_, POT_HIGH);
    srand((unsigned)time(NULL));   
    costs = costs_;
    start_x = start_x1;
    start_y = start_y1;
    end_x = end_x1;
    end_y = end_y1;
 //   ROS_WARN("into cal");
 //   std::fill(potential, potential + ns_, POT_HIGH);  // about potential
    //num_of_ptcs = 5000
    int arr_pot = 10;

	//costs is the whole costmap
    int start_i = toIndex(start_x, start_y);
    potential[start_i] = arr_pot;

    T.AddNode(start_x, start_y, start_i, start_i);       //father is itself
 //   ROS_WARN("start_x : %d %d",start_x,start_y);
  //  ROS_WARN("end_x: %d %d",end_x,end_y);
    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;
    end_tree_index = 0;  // store the end node index
//    ROS_WARN("into while");
    while (true) {
        int px = MyRand(0, nx_);
        int py = MyRand(0, ny_);
        if(m_use_goal_guide){
            double random = MyRand(0,1000) / (float)1000;
            if(random < guide_rate){
                int dx = ((end_x-guide_radius)<=0) ? (10) : (end_x-guide_radius);
                int ux = ((guide_radius+end_x) >= nx_) ? (nx_-10) : (guide_radius+end_x);
                int dy = ((end_y-guide_radius)<=0) ? (10) : (end_y-guide_radius);
                int uy = ((end_y+guide_radius)>=ny_) ? (ny_ - 10) : (end_y+guide_radius);
                px = MyRand(dx, ux);
                py = MyRand(dy, uy);
            }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
            }
        }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
        }
        //find_nearest node
        int m_index = T.Find_Near(px, py);
        int cx = T.Nodes_[m_index].x_;
        int cy = T.Nodes_[m_index].y_;

        ROS_ERROR("%d",toIndex(px, py));
        ROS_ERROR("%d",is_legal(toIndex(px, py)));
        if(is_legal(toIndex(px, py))){  // point legal
            if(is_not_col(cx, cy, px, py)){  // no obstacle between them
                int new_index = toward(cx, cy, px, py); // toward one step;
                int ccx = new_index % nx_, ccy = new_index / nx_;
                if(!is_legal(new_index))    continue;
                if(!is_not_col(cx, cy, ccx, ccy))    continue;
                T.AddNode(ccx, ccy, new_index, m_index);
                // if(!T.IfExist(new_index)){
                //     T.AddNode(ccx, ccy, new_index, m_index);
                // }else{
                //     continue;
                // }
                if(T.length >= T.max_length/10){
                    return false;
                }
                potential[new_index] = arr_pot;
                int dx = abs(ccx - end_x);
                int dy = abs(ccy - end_y);
                if(dx < arrive_offset && dy < arrive_offset){
                    T.AddNode(end_x, end_y, goal_i, T.length - 1);  // add endnode to tree
                    end_tree_index = T.length - 1;
                    potential[goal_i] = arr_pot;
                    break;            // arrive
                }
            }
        }
    }
   return true;
}
bool RRTExpansion::cut_bridge(){
    Node * cur = &T.Nodes_[T.length - 1];
    Node * fa = &T.Nodes_[T.length - 1];
    Node * grandpa;
    int flag = 1;
    while(true){
        if(flag == 0){
            break;
        }
        flag = 0;
        ROS_INFO("into improve");
        cur = &T.Nodes_[T.length - 1];
        fa = &T.Nodes_[cur->father_];
        grandpa = &T.Nodes_[fa->father_];
        while(T.Nodes_[cur->father_].index_ != toIndex(start_x,start_y)){
            if(is_not_col(grandpa->x_,grandpa->y_,cur->x_,cur->y_)){
                cur->father_ = fa->father_;
                fa = &T.Nodes_[cur->father_];
                grandpa = &T.Nodes_[fa->father_];
                flag = 1;
                ROS_WARN("cut");
            }else{
                cur = &T.Nodes_[cur->father_];
                fa = &T.Nodes_[cur->father_];
                grandpa = &T.Nodes_[fa->father_];
            }
        }
    }
        cur = &T.Nodes_[T.length - 1];
        fa = &T.Nodes_[cur->father_];
        grandpa = &T.Nodes_[fa->father_];
        while(cur->index_ != toIndex(start_x,start_y)){
            if(is_not_col(grandpa->x_,grandpa->y_,cur->x_,cur->y_)){
                cur->father_ = fa->father_;
                fa = &T.Nodes_[cur->father_];
                grandpa = &T.Nodes_[fa->father_];
            }else{
                cur = &T.Nodes_[cur->father_];
                fa = &T.Nodes_[cur->father_];
                grandpa = &T.Nodes_[fa->father_];
            }
        }

        ROS_INFO("after improve");
        ROS_INFO("length: %d",T.length);
}

bool RRTExpansion::single_rrt_star(unsigned char* costs_, double start_x1, double start_y1, double end_x1, double end_y1,
                                        int num_of_ptcs, float* potential){
    
    std::fill(potential, potential + ns_, POT_HIGH);
    srand((unsigned)time(NULL));   
    costs = costs_;
    start_x = start_x1;
    start_y = start_y1;
    end_x = end_x1;
    end_y = end_y1;
 //   ROS_WARN("into cal");
 //   std::fill(potential, potential + ns_, POT_HIGH);  // about potential
    //num_of_ptcs = 5000
    int arr_pot = 10;

    //costs is the whole costmap
    int start_i = toIndex(start_x, start_y);
    potential[start_i] = arr_pot;

    T.AddNode(start_x, start_y, start_i, start_i);       //father is itself
 //   ROS_WARN("start_x : %d %d",start_x,start_y);
  //  ROS_WARN("end_x: %d %d",end_x,end_y);
    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;
    end_tree_index = 0;  // store the end node index
//    ROS_WARN("into while");
    while (true) {
        int px = MyRand(0, nx_);
        int py = MyRand(0, ny_);
        if(m_use_goal_guide){
            double random = MyRand(0,1000) / (float)1000;
            if(random < guide_rate){
                int dx = ((end_x-guide_radius)<=0) ? (10) : (end_x-guide_radius);
                int ux = ((guide_radius+end_x) >= nx_) ? (nx_-10) : (guide_radius+end_x);
                int dy = ((end_y-guide_radius)<=0) ? (10) : (end_y-guide_radius);
                int uy = ((end_y+guide_radius)>=ny_) ? (ny_ - 10) : (end_y+guide_radius);
                px = MyRand(dx, ux);
                py = MyRand(dy, uy);
            }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
            }
        }else{
            px = MyRand(0, nx_);
            py = MyRand(0, ny_);
        }
        //find_nearest node
        int m_index = T.Find_Near(px, py);
        int cx = T.Nodes_[m_index].x_;
        int cy = T.Nodes_[m_index].y_;

        ROS_ERROR("%d",toIndex(px, py));
        ROS_ERROR("%d",is_legal(toIndex(px, py)));
        if(is_legal(toIndex(px, py))){  // point legal
            if(is_not_col(cx, cy, px, py)){  // no obstacle between them
                int new_index = toward(cx, cy, px, py); // toward one step;
                int ccx = new_index % nx_, ccy = new_index / nx_;
                if(!T.IfExist(new_index)){
                    ROS_WARN("into if3");
                    ROS_WARN("%d,%d,%d,%d",ccx, ccy, new_index, m_index);
                    T.AddNode(ccx, ccy, new_index, m_index);
                    ROS_WARN("done add");
                    //star step
                    int star_cost = T.calculateCost(T,T.length - 1, start_i);
                    ROS_WARN("into for");
                    for(int i = 0,k = 0; i < T.length - 1; i++){
                        ROS_ERROR("TIMES:%d",i);
                        int cur_dis = abs(T.Nodes_[i].x_ - ccx) + abs(T.Nodes_[i].y_ - ccy);
                        if( cur_dis <= explore_radius){
                            T.Nodes_[T.length - 1].father_ = i;
                            if(star_cost >= T.calculateCost(T,T.length - 1, start_i)){
                                star_cost = T.calculateCost(T,T.length - 1, start_i);
                                m_index = i;
                            }else{
                                T.Nodes_[T.length - 1].father_ = m_index;
                            }
                        }
                    }
                    // step one
                    // m_index -> min cost id
                    for(int i = 0,k = 0; i < T.length - 1; i++){
                        if(i == m_index) continue;
                        int cur_dis = abs(T.Nodes_[i].x_ - ccx) + abs(T.Nodes_[i].y_ - ccy);
                        if( cur_dis <= explore_radius){
                            int star_n_cost = T.calculateCost(T, i, start_i);
                            if(star_n_cost >= T.calculateCost(T,T.length - 1, start_i) + abs(T.Nodes_[i].x_ - T.Nodes_[T.length - 1].x_)*
                            abs(T.Nodes_[i].x_ - T.Nodes_[T.length - 1].x_) + abs(T.Nodes_[i].y_ - T.Nodes_[T.length - 1].y_)*
                            abs(T.Nodes_[i].y_ - T.Nodes_[T.length - 1].y_)) {
                                T.Nodes_[i].father_ = T.length - 1;
                            }
                        }
                    }
                }

                potential[new_index] = arr_pot;
                int dx = abs(ccx - end_x);
                int dy = abs(ccy - end_y);
                if(dx < arrive_offset && dy < arrive_offset){
                    T.AddNode(end_x, end_y, goal_i, T.length - 1);  // add endnode to tree
                    end_tree_index = T.length - 1;
                    potential[goal_i] = arr_pot;
                    break;            // arrive
                }
            }
        }
    }
   return true;
}

} //end namespace global_planner
