#pragma once
#include "surgical_units.hpp"

/*
  Creates and returns a shared_ptr to a hole.
*/
Hole::Ptr make_hole (){ 
  Hole::Ptr hole (new Hole());
  return hole;
}

Hole::Ptr make_hole (uint8_t H, uint8_t S, uint8_t V,
	       uint8_t Hstd, uint8_t Sstd, uint8_t Vstd){

  Vector3f temp;
  Hole::Ptr hole (new Hole(temp, H, S, V, Hstd, Sstd, Vstd, 0, 0));
  return hole;
}

/*
  Creates and returns a shared_ptr to a cut.
*/
Cut::Ptr make_cut (){
  //CutPtr cut (new Cut(xyzs, H, S, V, Hstd, Sstd, Vstd, x_IDs, y_IDs));
  //temp:
  Cut::Ptr cut (new Cut());
  return cut;
}

Cut::Ptr make_cut (uint8_t H, uint8_t S, uint8_t V,
	       uint8_t Hstd, uint8_t Sstd, uint8_t Vstd){

  vector<Vector3f> temp;
  vector<int> temp_int;
  Cut::Ptr cut (new Cut(temp, H, S, V, Hstd, Sstd, Vstd, temp_int, temp_int));
  return cut;
}

/*
  Creates and returns a shared_ptr to a suturing needle.
*/
Needle::Ptr make_needle (){
  Needle::Ptr needle (new Needle());
  return needle;
}
