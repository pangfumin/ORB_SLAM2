#include <iostream>
#include "../g2o/types/types_six_dof_expmap.h"
#include "../g2o/core/base_binary_edge.h"
#include "../g2o/core/block_solver.h"
#include "../g2o/core/optimization_algorithm_levenberg.h"
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/core/optimization_algorithm_dogleg.h>
#include "../g2o/solvers/linear_solver_dense.h"
#include "../g2o/solvers/linear_solver_eigen.h"
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>


int main() {

    // simulate
    Eigen::Isometry3d T_WC, est_T_WC;
    est_T_WC.setIdentity();
    T_WC.setIdentity();

    est_T_WC.matrix().topLeftCorner(3,3) = Eigen::Quaterniond(0.2,0,0.3,-0.4).normalized().toRotationMatrix();
    est_T_WC.matrix().topRightCorner(3,1) = Eigen::Vector3d(0.3,0.3,0);
    std::cout<<"est_T_WC: "<< std::endl<< est_T_WC.matrix()<<std::endl;


    // create optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // add vertex and edge
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    int index = 0;
    v->setId( index );
    g2o::SE3Quat est(est_T_WC.matrix().topLeftCorner(3,3), est_T_WC.matrix().topRightCorner(3,1));
    v->setEstimate(est);
    optimizer.addVertex(v);

    g2o::EdgeSE3PoseConstraint* e = new g2o::EdgeSE3PoseConstraint();

    e->setId( 0 );
    g2o::SE3Quat meas(T_WC.matrix().topLeftCorner(3,3), T_WC.matrix().topRightCorner(3,1));
    e->setMeasurement(meas);
    e->setVertex( 0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)) );
    e->setInformation(Eigen::Matrix<double,6,6>::Identity());

    optimizer.addEdge(e);


    // optmizing
    std::cout<<"prepare optimizing ..."<<std::endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    std::cout<<"calling optimizing ..."<<std::endl;
    optimizer.optimize(3);

    std::cout<<"est: "<<v->estimate()<<std::endl;


    return 0;
}