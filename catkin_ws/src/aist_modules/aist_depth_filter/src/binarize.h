/*
 *  $BJ?@.(B14-19$BG/!JFH!K;:6H5;=QAm9g8&5f=j(B $BCx:n8"=jM-(B
 *
 *  $BAO:n<T!'?"<G=SIW(B
 *
 *  $BK\%W%m%0%i%`$O!JFH!K;:6H5;=QAm9g8&5f=j$N?&0w$G$"$k?"<G=SIW$,AO:n$7!$(B
 *  $B!JFH!K;:6H5;=QAm9g8&5f=j$,Cx:n8"$r=jM-$9$kHkL)>pJs$G$9!%Cx:n8"=jM-(B
 *  $B<T$K$h$k5v2D$J$7$KK\%W%m%0%i%`$r;HMQ!$J#@=!$2~JQ!$Bh;0<T$X3+<($9$k(B
 *  $BEy$N9T0Y$r6X;_$7$^$9!%(B
 *
 *  $B$3$N%W%m%0%i%`$K$h$C$F@8$8$k$$$+$J$kB;32$KBP$7$F$b!$Cx:n8"=jM-<T$*(B
 *  $B$h$SAO:n<T$O@UG$$rIi$$$^$;$s!#(B
 *
 *  Copyright 2002-2007.
 *  National Institute of Advanced Industrial Science and Technology (AIST)
 *
 *  Creator: Toshio UESHIBA
 *
 *  [AIST Confidential and all rights reserved.]
 *  This program is confidential. Any using, copying, changing or
 *  giving any information concerning with this program to others
 *  without permission by the copyright holder are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holder or the creator are not responsible for any
 *  damages caused by using this program.
 *
 *  $Id: binarize.h,v 1.1 2008-11-07 05:34:21 ueshiba Exp $
 */
/*!
  \file		binarize.h
  \brief	$BBgDE$N(B2$BCM2=%"%k%4%j%:%`$N<BAu(B
*/
#include <algorithm>

namespace TU
{
//! $BBgDE$N(B2$BCM2=%"%k%4%j%:%`$K$h$jM?$($i$l$?%G!<%?Ns$r(B2$BCM2=$9$k!%(B
/*!
  $B%G!<%?Ns$O>:=g$K%=!<%H$5$l!$ogCM$H$J$k%G!<%?$9$J$o$A8eH>It$N@hF,%G!<%?$,JV$5$l$k!%(B
  \param begin	$B%G!<%?Ns$N@hF,$r<($9H?I|;R(B
  \param end	$B%G!<%?Ns$NKvHx$N<!$r<($9H?I|;R(B
  \return	$BogCM$H$J$k%G!<%?$r<($9H?I|;R(B
*/
template <class Iterator> Iterator
binarize(Iterator begin, Iterator end)
{
  // $BMWAG?t$HJ?6QCM$r7W;;!%(B
    long	n = 0;
    double	mean = 0;
    for (Iterator iter = begin; iter != end; ++iter)
    {
	++n;
	mean += *iter;
    }
    mean /= n;

  // $B>:=g$K%=!<%H!%(B
    std::sort(begin, end);

  // $BBgDE$NH=JL4p=`$K$h$j:GE,$J$7$-$$CM$r7hDj!%(B
    Iterator	thresh = begin;
    long	nLow = 0;		// $B$7$-$$CM0J2<$NMWAG?t(B
    double	cumulationLow = 0;	// $B$7$-$$CM0J2<$NN_@QCM(B
    double	interVarianceMax = 0;	// $B%/%i%94VJ,;6$N:GBgCM(B
    for (Iterator iter = begin, head = begin; iter != end; ++iter)
    {
	if (*iter != *head)
	{
	    double	interVariance = cumulationLow - nLow * mean;
	    ((interVariance *= interVariance) /= nLow) /= (n - nLow);
	    if (interVariance > interVarianceMax)
	    {
		interVarianceMax = interVariance;
		thresh = iter;
	    }
	    head = iter;
	}

	++nLow;
	cumulationLow += *iter;
    }

    return thresh;
}

}
