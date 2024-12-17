package frc.robot.Sysid;

import org.ejml.simple.SimpleMatrix;

public class DataTable {
    public SimpleMatrix dataMatrix;
    public SimpleMatrix powerMatrix;
    private int nextRow;

  public DataTable(int nextRow, int totalRows, int totalColumns){
    this.dataMatrix = new SimpleMatrix(totalRows, totalColumns);
    this.powerMatrix = new SimpleMatrix(totalRows, 1);
    this.nextRow = nextRow;
    
  }

  public void updateRow(){
    this.nextRow++;
  }

  public int getRow(){
    return this.nextRow;
  }

  public SimpleMatrix solve(){
    return dataMatrix.solve(powerMatrix);
  }
}
