#pragma once
#include "surgical_units.hpp"

Hole::Ptr make_hole (){ 
  //HolePtr hole (new Hole(xyz, H, S, V, Hstd, Sstd, Vstd, x_ID, y_ID));
  //temp:
  Hole::Ptr hole (new Hole());
  return hole;
}

Cut::Ptr make_cut (){  
  //CutPtr cut (new Cut(xyzs, H, S, V, Hstd, Sstd, Vstd, x_IDs, y_IDs));
  //temp:
  Cut::Ptr cut (new Cut());
  return cut;
}

Suture::Ptr make_suture (){
  //SuturePtr suture (new Suture(H, S, V, Hstd, Sstd, Vstd));
  //temp:
  Suture::Ptr suture (new Suture());
  return suture;
}
