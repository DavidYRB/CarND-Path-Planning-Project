#include <iostream>
#include <vector>
#include <unordered_map>
using namespace std;

/* goal: find the root of f(x). definition of root: x* such that f(x*) = 0.
initial value, we want to 





*/

double func(double x){
  return x * x - 2;
}

float root() {
  // x, f(x)　are double
  // f(x) = x * x * x - 2 
  double left{1};
  double right{2};
  double left_result = func(left);
  double right_result = func(right);
  double mid{0};
  double mid_result{0};
  while(abs(left_result) > 0.0001 && abs(right_result) > 0.0001){
    mid = left + (right - left)/2;
    mid_result = func(mid);
    if(mid_result > 0){
      right = mid;
    }
    else if(mid_result < 0){
      left = mid;
    }
    else{
      return mid;
    }
    left_result = func(left);
    right_result = func(right);
  }
  return abs(left_result) < abs(left_result) ? left : right;
}

vector<vector<int>> twosumsToZero(const vector<int> &vec)
{
  vector<vector<int>> result;
  int size = vec.size();
  unordered_map<int, vector<int>> checkList;
  for(int i = 0; i < size; ++i){
    int curr = vec[i];
    int residule = 0 - curr;
    if(checkList.find(residule) != checkList.end()){
      for(auto index : checkList[residule]){
        cout << "curr index: " << i << " prev index : " << index << endl;
        vector<int> temp{index, i};
        result.push_back(temp);
      }
    }
    //if(checkList.find(curr) == checkList.end()){
    if(checkList.find(curr) != checkList.end()){
      vector<int> temp{i};
      cout << "1cur:" << curr << "  i:" << i << std::endl;
      checkList[curr] = temp;
    }
    else{
      cout << "2cur:" << curr << "  i:" << i << std::endl;
      checkList[curr].push_back(i);
    }
  }
  return result;
}


int main() {

  // std::cout << root() << std::endl;
  vector<int> vec{1,2,3,3,0,1,-3,-3,-1};
  auto res = twosumsToZero(vec);
  for(auto v : res) {
    cout << v[0] << "," << v[1] << std::endl;
  }
}
