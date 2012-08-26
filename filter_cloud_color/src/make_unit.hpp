#pragma once
#include "surgical_units.hpp"

/*
  Creats and returns a shared_ptr to a hole.
*/
Hole::Ptr make_hole (){ 
  //HolePtr hole (new Hole(xyz, H, S, V, Hstd, Sstd, Vstd, x_ID, y_ID));
  //temp:
  Hole::Ptr hole (new Hole());
  return hole;
}

/*
  Creats and returns a shared_ptr to a cut.
*/
Cut::Ptr make_cut (){  
  //CutPtr cut (new Cut(xyzs, H, S, V, Hstd, Sstd, Vstd, x_IDs, y_IDs));
  //temp:
  Cut::Ptr cut (new Cut());
  return cut;
}

/*
  Creats and returns a shared_ptr to a suture.
*/
Suture::Ptr make_suture (){
  //SuturePtr suture (new Suture(H, S, V, Hstd, Sstd, Vstd));
  //temp:
  Suture::Ptr suture (new Suture());
  return suture;
}
