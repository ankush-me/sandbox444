#pragma once
#include "surgical_units.hpp"

/*
  Creates and returns a shared_ptr to a hole.
*/
Hole::Ptr make_hole (){ 
  //HolePtr hole (new Hole(xyz, H, S, V, Hstd, Sstd, Vstd, x_ID, y_ID));
  //temp:
  Hole::Ptr hole (new Hole());
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

/*
  Creates and returns a shared_ptr to a suturing needle.
*/
Needle::Ptr make_needle (){
  //NeedlePtr needle (new Needle(H, S, V, Hstd, Sstd, Vstd));
  //temp:
  Needle::Ptr needle (new Needle());
  return needle;
}
