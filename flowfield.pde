class FlowField {
  PVector[][] field; // 2D array of PVectors
  int cols, rows; // number of rows and columns in the grid
  int resolution; // resolution of the grid

  FlowField(int r) {
    resolution = r;
    
    cols = width / resolution;
    rows = height / resolution;
    
    field = new PVector[cols][rows];
    
    // use 3D Perlin noise to assign the vectors of the flow field
    float xoff = 0;
    for (int i = 0; i<cols; i++){
      float yoff = 0;
      for (int j = 0; j<rows; j++){
        float theta = map(noise(xoff, yoff), 0, 1, 0, TWO_PI);
        field[i][j] = new PVector(cos(theta), sin(theta));
        yoff += 0.1;
      }
    xoff += 0.1;
    }
  }
  
  // function to return the flow field in the vehicle location
  PVector lookup(PVector lookup){
    // constrain the variables to be inside the window
    int column = int(constrain(lookup.x/resolution, 0, cols-1));
    int row = int(constrain(lookup.y/resolution, 0, rows-1));
    
    return field[column][row].get(); // we use get to return a copy of the vector
  }

}
