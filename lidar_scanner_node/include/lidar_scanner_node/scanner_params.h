#ifndef SCANNER_PARAMS_H
#define SCANNER_PARAMS_H

struct scanner_params
{
  double theta_span, phi_span, los_variance, orthogonal_variance, max_incidence_angle, max_distance;
  int theta_points, phi_points;
};

#endif // SCANNER_PARAMS_H
