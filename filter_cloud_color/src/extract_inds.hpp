#pragma once
#include <vector>

using namespace std;

/*
  Checks if element is present in vector. 
  Assumes '==' operator is defined for vectype.
*/
template <typename vectype>
bool hasElement (vector<vectype> * vec, vectype element) {
  for (int i = 0; i < vec->size(); i++) {
    if (vec->at(i)==element)
      return true;
  }
  return false;
}

/* 
   Extracts individual digits as indices for holes and cuts.
   Assumes there are less than 10 cuts/holes.
*/
vector<int> extractInds (int n) {
  vector<int> inds;
  
  if (n<1)
    inds.push_back(0);
  else {
    while (n>0) {
      int d = n%10;
      if (!hasElement(&inds, d))
	inds.push_back(d);
      n = (n - d)/10;
    }
  }

  return inds;
}
