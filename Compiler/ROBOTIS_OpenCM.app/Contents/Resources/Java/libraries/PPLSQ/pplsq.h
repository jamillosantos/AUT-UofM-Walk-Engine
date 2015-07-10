/*  pplsq.c    CCMATH mathematics library source code.
 *
 *  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
 *  This code may be redistributed under the terms of the GNU library
 *  public license (LGPL). ( See the lgpl.license file for details.)
 * ------------------------------------------------------------------------
 */
#include <stdlib.h>
#include "orpol.h"

#ifndef PPLSQ
void plsq(double *x,double *y,int n,Opol *c,double *ss,int m);
void psqcf(double *pc,Opol *cf,int m);

double pplsq(double *x,double *y,int n,double *bc,int m)
{
  Opol *c;
  double *ss,sq;
  c=(Opol *)calloc(m,sizeof(Opol));
  ss=(double *)calloc(m,sizeof(double));
  plsq(x,y,n,c,ss,m);
  psqcf(bc,c,m);
  sq=ss[m-1];
  free(c); free(ss);
  return sq;
}
void psqcf(double *b,Opol *c,int m)
{ int i,j,k; double *sm,*s,u,v;
  if(m>1){
    sm=(double *)calloc(m*m,sizeof(double));
    sm[0]=sm[m+1]=1.; sm[1]= -c[0].df;
    for(i=2; i<m ;++i){ k=i-1;
      for(j=0,s=sm+i,v=0.; j<i ;++j,s+=m){
	*s=v-c[k].df* *(s-1)-c[k].hs* *(s-2);
	v= *(s-1);
       }
      *s=1.;
     }
    for(i=0; i<m ;++i){
      for(j=i,v=0.,s=sm+(m+1)*i; j<m ;++j) v+= *s++ *c[j].cf;
      b[i]=v;
     }
    free(sm);
   }
  else b[0]=c[0].cf;
}
void plsq(double *x,double *y,int n,Opol *cf,double *ssq,int m)
{ double *pm,*e,*p,*q;
  double f,s,t,u,w,tp;
  int i,j,k,l;
  pm=(double *)calloc(3*n,sizeof(double));
  for(i=0,w=u=0.,e=pm,p=e+n; i<n ;++i)
  {
    w+=y[i]; u+=x[i]; *p++ =1.; *e++ =y[i];
  }
  cf[0].hs=tp=(double)n; cf[0].cf=w/tp; cf[0].df=u/tp; 
  for(k=1; k<m ;++k){ l=k-1;
    for(j=0,s=t=u=w=0.,e=pm,p=e+n,q=p+n; j<n ;++j){
      *e-= cf[l].cf* *p; s+= *e* *e;
      f=(x[j]-cf[l].df)* *p- cf[l].hs* *q;
      *q++ = *p; *p++ =f; w+=f* *e++;
      t+=(f*=f); if(k<m-1) u+=x[j]*f;
     }
    ssq[l]=s; cf[k].cf=w/t; if(k<m-1) cf[k].df=u/t;
    cf[k].hs=t/tp; tp=t;
   }
  l=m-1; t=cf[l].cf; cf[l].df=0.;
  for(j=0,e=pm,p=e+n,s=0.; j<n ;++j,++e){
    *e-=t* *p++; s+= *e* *e; 
   }
  ssq[l]=s;
  free(pm);
}
#endif
