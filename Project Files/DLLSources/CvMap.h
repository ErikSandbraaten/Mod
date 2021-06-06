#pragma once

#ifndef CIV4_MAP_H
#define CIV4_MAP_H

//
//	FILE:	 CvMap.h
//	AUTHOR:  Soren Johnson
//	PURPOSE: Game map class
//-----------------------------------------------------------------------------
//	Copyright (c) 2004 Firaxis Games, Inc. All rights reserved.
//-----------------------------------------------------------------------------
//


#include "CvArea.h"
#include "CvPlot.h"


class FAStar;


inline int coordRange(int iCoord, int iRange, bool bWrap)
{
	if (bWrap)
	{
		if (iRange != 0)
		{
			if (iCoord < 0 )
			{
				return (iRange + (iCoord % iRange));
			}
			else if (iCoord >= iRange)
			{
				return (iCoord % iRange);
			}
		}
	}

	return iCoord;
}


//
// holds initialization info
//
struct CvMapInitData
{
	int m_iGridW;						// in game plots
	int m_iGridH;						// in game plots
	int m_iTopLatitude;
	int m_iBottomLatitude;

	bool m_bWrapX;
	bool m_bWrapY;

	CvMapInitData(int iGridW=0, int iGridH=0, int iTopLatitude=90, int iBottomLatitude=-90, bool bWrapX=false, bool bWrapY=false) :
		m_iGridH(iGridH),m_iGridW(iGridW),m_iTopLatitude(iTopLatitude),m_iBottomLatitude(iBottomLatitude),m_bWrapY(bWrapY),m_bWrapX(bWrapX)
	{ }
};


//
// CvMap
//
class CvSelectionGroup;
class CvMap
{

	friend class CyMap;

public:
	// 4 | 4 | 3 | 3 | 3 | 4 | 4
	// -------------------------
	// 4 | 3 | 2 | 2 | 2 | 3 | 4
	// -------------------------
	// 3 | 2 | 1 | 1 | 1 | 2 | 3
	// -------------------------
	// 3 | 2 | 1 | 0 | 1 | 2 | 3
	// -------------------------
	// 3 | 2 | 1 | 1 | 1 | 2 | 3
	// -------------------------
	// 4 | 3 | 2 | 2 | 2 | 3 | 4
	// -------------------------
	// 4 | 4 | 3 | 3 | 3 | 4 | 4
	//
	// Returns the distance between plots according to the pattern above...
	inline int plotDistance(int iX1, int iY1, int iX2, int iY2) const
	{
		int iDX = xDistance(iX1, iX2);
		int iDY = yDistance(iY1, iY2);
		return max(iDX, iDY) + (min(iDX, iDY) / 2);
	}

	// K-Mod, plot-to-plot alias for convenience:
	inline int plotDistance(const CvPlot* plot1, const CvPlot* plot2) const
	{
		return plotDistance(
				plot1->getX(), plot1->getY(),
				plot2->getX(), plot2->getY());
	}
	// K-Mod end

	// 3 | 3 | 3 | 3 | 3 | 3 | 3
	// -------------------------
	// 3 | 2 | 2 | 2 | 2 | 2 | 3
	// -------------------------
	// 3 | 2 | 1 | 1 | 1 | 2 | 3
	// -------------------------
	// 3 | 2 | 1 | 0 | 1 | 2 | 3
	// -------------------------
	// 3 | 2 | 1 | 1 | 1 | 2 | 3
	// -------------------------
	// 3 | 2 | 2 | 2 | 2 | 2 | 3
	// -------------------------
	// 3 | 3 | 3 | 3 | 3 | 3 | 3
	//
	// Returns the distance between plots according to the pattern above...
	inline int stepDistance(int iX1, int iY1, int iX2, int iY2) const
	{
		return max(xDistance(iX1, iX2), yDistance(iY1, iY2));
	}

	// K-Mod, plot-to-plot alias for convenience:
	inline int stepDistance(const CvPlot* plot1, const CvPlot* plot2) const
	{
		return stepDistance(
			plot1->getX(), plot1->getY(),
			plot2->getX(), plot2->getY());
	} // K-Mod end

	inline CvPlot* plotDirection(int iX, int iY, DirectionTypes eDirection) const
	{
		if (eDirection == NO_DIRECTION)
			return plotValidXY(iX, iY);
		// advc.opt: Don't check for INVALID_PLOT_COORD
		else return plotValidXY(
			iX + GC.getPlotDirectionX()[eDirection],
			iY + GC.getPlotDirectionY()[eDirection]);
	}

	inline CvPlot* plotCardinalDirection(int iX, int iY, CardinalDirectionTypes eCardinalDirection) const
	{
		// advc.opt: Don't check for INVALID_PLOT_COORD
		return plotValidXY(
			iX + GC.getPlotCardinalDirectionX()[eCardinalDirection],
			iY + GC.getPlotCardinalDirectionY()[eCardinalDirection]);
	}

	inline CvPlot* plotXY(int iX, int iY, int iDX, int iDY) const
	{
		// advc.opt: Don't check for INVALID_PLOT_COORD
		return plotValidXY(iX + iDX, iY + iDY);
	}
	// K-Mod start
	inline CvPlot* plotXY(const CvPlot* pPlot, int iDX, int iDY) const
	{
		return plotXY(pPlot->getX(), pPlot->getY(), iDX, iDY);
	} // K-Mod end

	inline DirectionTypes directionXY(int iDX, int iDY) const
	{
		if (abs(iDX) > DIRECTION_RADIUS || abs(iDY) > DIRECTION_RADIUS)
			return NO_DIRECTION;
		else return GC.getXYDirection(iDX + DIRECTION_RADIUS, iDY + DIRECTION_RADIUS);
	}

	inline DirectionTypes directionXY(CvPlot const& kFromPlot, CvPlot const& kToPlot) const // advc: take params as references
	{
		return directionXY(
			dxWrap(kToPlot.getX() - kFromPlot.getX()),
			dyWrap(kToPlot.getY() - kFromPlot.getY()));
	}

	inline int dxWrap(int iDX) const
	{
		return wrapCoordDifference(iDX, getGridWidth(), isWrapXInternal());
	}

	inline int dyWrap(int iDY) const
	{
		return wrapCoordDifference(iDY, getGridHeight(), isWrapYInternal());
	}

	inline int xDistance(int iFromX, int iToX) const
	{
		return coordDistance(iFromX, iToX, getGridWidth(), isWrapXInternal());
	}

	inline int yDistance(int iFromY, int iToY) const
	{
		return coordDistance(iFromY, iToY, getGridHeight(), isWrapYInternal());
	}
	inline CvPlot* plotCity(int iX, int iY, CityPlotTypes ePlot) const						// Exposed to Python (CyGameCoreUtils.py)
	{	// advc.enum: 3rd param was int
		// advc.opt: Don't check for INVALID_PLOT_COORD
		return plotValidXY(iX + GC.getCityPlotX()[ePlot], iY + GC.getCityPlotY()[ePlot]);
	}
	CityPlotTypes plotCityXY(int iDX, int iDY) const // advc.enum: return CityPlotTypes		// Exposed to Python (CyGameCoreUtils.py)
	{
		if (abs(iDX) > CITY_PLOTS_RADIUS || abs(iDY) > CITY_PLOTS_RADIUS)
			return NO_CITY_PLOT; // advc.enum
		return GC.getXYCityPlot(iDX + CITY_PLOTS_RADIUS, iDY + CITY_PLOTS_RADIUS);
	}
	// advc: 1st param (CvCity*) replaced with two ints - to allow hypothetical city sites

	inline CityPlotTypes plotCityXY(int iCityX, int iCityY, CvPlot const& kPlot) const		// Exposed to Python (CyGameCoreUtils.py)
	{
		return plotCityXY(dxWrap(kPlot.getX() - iCityX), dyWrap(kPlot.getY() - iCityY));
	}
	// advc:
	inline bool adjacentOrSame(CvPlot const& kFirstPlot, CvPlot const& kSecondPlot) const
	{
		return (stepDistance(&kFirstPlot, &kSecondPlot) <= 1);
	}

	inline int plotCityXY(int iDX, int iDY)
	{
		if ((abs(iDX) > CITY_PLOTS_RADIUS) || (abs(iDY) > CITY_PLOTS_RADIUS))
		{
			return -1;
		}
		else
		{
			return GC.getXYCityPlot((iDX + CITY_PLOTS_RADIUS), (iDY + CITY_PLOTS_RADIUS));
		}
	}

private: // Auxiliary functions
		 /*	These look too large and branchy for inlining, but the keywords do seem
		 to improve performance a little bit. Were also present in BtS. */

	inline int coordDistance(int iFrom, int iTo, int iRange, bool bWrap) const
	{
		if (bWrap && abs(iFrom - iTo) > iRange / 2)
			return iRange - abs(iFrom - iTo);
		return abs(iFrom - iTo);
	}

	inline int wrapCoordDifference(int iDiff, int iRange, bool bWrap) const
	{
		if (!bWrap)
			return iDiff;
		if (iDiff > iRange / 2)
			return iDiff - iRange;
		else if (iDiff < -(iRange / 2))
			return iDiff + iRange;
		return iDiff;
	}

	inline int coordRange(int iCoord, int iRange, bool bWrap) const
	{
		if (!bWrap || iRange == 0)
			return iCoord;
		if (iCoord < 0)
			return (iRange + (iCoord % iRange));
		else if (iCoord >= iRange)
			return (iCoord % iRange);
		return iCoord;
	}
	// </advc.make>

public:
	CvMap();
	virtual ~CvMap();

	DllExport void init(CvMapInitData* pInitData=NULL);
	DllExport void setupGraphical();
	DllExport void reset(CvMapInitData* pInitData);

protected:

	void uninit();
	void setup();

public:
	DllExport void erasePlots();
	void setRevealedPlots(TeamTypes eTeam, bool bNewValue, bool bTerrainOnly = false);
	void setAllPlotTypes(PlotTypes ePlotType);

	void doTurn();

	DllExport void updateFlagSymbols();

	DllExport void updateFog();
	void updateVisibility();
	DllExport void updateSymbolVisibility();
	void updateSymbols();
	DllExport void updateMinimapColor();
	void updateSight(bool bIncrement);
	DllExport void updateCenterUnit();
	void updateWorkingCity();
	void updateMinOriginalStartDist(CvArea* pArea);
	void updateYield();
	void updateCulture();

	void verifyUnitValidPlot();

	CvPlot* syncRandPlot(int iFlags = 0, int iArea = -1, int iMinUnitDistance = -1, int iTimeout = 100);

	DllExport CvCity* findCity(int iX, int iY, PlayerTypes eOwner = NO_PLAYER, TeamTypes eTeam = NO_TEAM, bool bSameArea = true, bool bCoastalOnly = false, TeamTypes eTeamAtWarWith = NO_TEAM, DirectionTypes eDirection = NO_DIRECTION, CvCity* pSkipCity = NULL);
	CvSelectionGroup* findSelectionGroup(int iX, int iY, PlayerTypes eOwner = NO_PLAYER, bool bReadyToSelect = false);

	CvArea* findBiggestArea(bool bWater);

	int getMapFractalFlags();
	bool findWater(CvPlot* pPlot, int iRange, bool bFreshWater);

	DllExport bool isPlot(int iX, int iY) const;
#ifdef _USRDLL
	inline int isPlotINLINE(int iX, int iY) const
	{
		return ((iX >= 0) && (iX < getGridWidthINLINE()) && (iY >= 0) && (iY < getGridHeightINLINE()));
	}
#endif
	DllExport int numPlots() const;
#ifdef _USRDLL
	inline int numPlotsINLINE() const
	{
		return getGridWidthINLINE() * getGridHeightINLINE();
	}
#endif
	DllExport int plotNum(int iX, int iY) const;
#ifdef _USRDLL
	inline int plotNumINLINE(int iX, int iY) const
	{
		return ((iY * getGridWidthINLINE()) + iX);
	}
#endif
	int plotX(int iIndex) const;
	int plotY(int iIndex) const;

	int pointXToPlotX(float fX);
	DllExport float plotXToPointX(int iX);

	int pointYToPlotY(float fY);
	DllExport float plotYToPointY(int iY);

	float getWidthCoords();
	float getHeightCoords();
	int maxPlotDistance();
	int maxStepDistance();

	DllExport int getGridWidth() const;
#ifdef _USRDLL
	inline int getGridWidthINLINE() const
	{
		return m_iGridWidth;
	}
#endif
	DllExport int getGridHeight() const;
#ifdef _USRDLL
	inline int getGridHeightINLINE() const
	{
		return m_iGridHeight;
	}
#endif
	int getLandPlots();
	void changeLandPlots(int iChange);
	int getOwnedPlots();
	void changeOwnedPlots(int iChange);
	int getTopLatitude();
	int getBottomLatitude();

	int getNextRiverID();
	void incrementNextRiverID();

	DllExport bool isWrapX();
#ifdef _USRDLL
	inline bool isWrapXINLINE() const
	{
		return m_bWrapX;
	}
#endif
	DllExport bool isWrapY();
#ifdef _USRDLL
	inline bool isWrapYINLINE() const
	{
		return m_bWrapY;
	}
#endif
	DllExport bool isWrap();
#ifdef _USRDLL
	inline bool isWrapINLINE() const
	{
		return m_bWrapX || m_bWrapY;
	}
#endif

	inline bool isWrapXInternal() const { return m_bWrapX; } // advc.inl
	inline bool isWrapYInternal() const { return m_bWrapY; } // advc.inl

	DllExport WorldSizeTypes getWorldSize();
	ClimateTypes getClimate();
	SeaLevelTypes getSeaLevel();

	int getNumCustomMapOptions();
	CustomMapOptionTypes getCustomMapOption(int iOption);

	int getNumBonuses(BonusTypes eIndex);
	void changeNumBonuses(BonusTypes eIndex, int iChange);
	int getNumBonusesOnLand(BonusTypes eIndex);
	void changeNumBonusesOnLand(BonusTypes eIndex, int iChange);
	DllExport CvPlot* plotByIndex(int iIndex) const;
#ifdef _USRDLL
	inline CvPlot* plotByIndexINLINE(int iIndex) const
	{
		return (((iIndex >= 0) && (iIndex < (getGridWidthINLINE() * getGridHeightINLINE()))) ? &(m_pMapPlots[iIndex]) : NULL);
	}
#endif
	DllExport CvPlot* plot(int iX, int iY) const;
#ifdef _USRDLL
	__forceinline CvPlot* plotINLINE(int iX, int iY) const
	{
		if ((iX == INVALID_PLOT_COORD) || (iY == INVALID_PLOT_COORD))
		{
			return NULL;
		}
		int iMapX = coordRange(iX, getGridWidthINLINE(), isWrapXINLINE());
		int iMapY = coordRange(iY, getGridHeightINLINE(), isWrapYINLINE());
		return ((isPlotINLINE(iMapX, iMapY)) ? &(m_pMapPlots[plotNumINLINE(iMapX, iMapY)]) : NULL);
	}
	__forceinline CvPlot* plotSoren(int iX, int iY) const // advc.inl: Renamed from plotSorenINLINE
	{
		if (iX == INVALID_PLOT_COORD || iY == INVALID_PLOT_COORD)
			return NULL;
		FAssert(isPlot(iX, iY)); // advc: Assertion added
		return &(m_pMapPlots[plotNum(iX, iY)]);
	} // <advc.inl> Even faster and less confusingly named; replacing the above in most places.
	__forceinline CvPlot& getPlot(int x, int y) const
	{
		FAssert(isPlot(x, y));
		return m_pMapPlots[plotNum(x, y)];
	} // </advc.inl>
	/*	advc.opt: Yet another plot getter. Checks coordRange but not INVALID_PLOT_COORD.
		For functions that compute x,y as an offset from a (valid) plot -
		not plausible that the new coordinates would equal INVALID_PLOT_COORD.
		'inline' tested - faster without it. */
	CvPlot* plotValidXY(int iX, int iY) const
	{
		int iMapX = coordRange(iX, getGridWidth(), isWrapXInternal());
		int iMapY = coordRange(iY, getGridHeight(), isWrapYInternal());
		return (isPlot(iMapX, iMapY) ? &m_pMapPlots[plotNum(iMapX, iMapY)] : NULL);
	}

#endif
	DllExport CvPlot* pointToPlot(float fX, float fY);
	int getIndexAfterLastArea();
	int getNumAreas();
	int getNumLandAreas();

	CvArea* getArea(int iID);
	CvArea* addArea();
	void deleteArea(int iID);
	// iteration
	CvArea* firstArea(int *pIterIdx, bool bRev=false);
	CvArea* nextArea(int *pIterIdx, bool bRev=false);

	void recalculateAreas();

	void resetPathDistance();	
	// Super Forts begin *canal* *choke*
	//int calculatePathDistance(CvPlot *pSource, CvPlot *pDest); //original
	int calculatePathDistance(CvPlot *pSource, CvPlot *pDest, CvPlot *pInvalidPlot = NULL);	// Exposed to Python
	void calculateCanalAndChokePoints();	// Exposed to Python
	// Super Forts end

	// Serialization:
	virtual void read(FDataStreamBase* pStream);
	virtual void write(FDataStreamBase* pStream);
	void rebuild(int iGridW, int iGridH, int iTopLatitude, int iBottomLatitude, bool bWrapX, bool bWrapY, WorldSizeTypes eWorldSize, ClimateTypes eClimate, SeaLevelTypes eSeaLevel, int iNumCustomMapOptions, CustomMapOptionTypes * eCustomMapOptions);

	void writeDesyncLog(FILE *f);

	char getCityCatchmentRadius() const;
	void setCityCatchmentRadius(int iSetting);

protected:

	void resetSavedData();
	void read(CvSavegameReader reader);
	void write(CvSavegameWriter writer);

	int m_iGridWidth;
	int m_iGridHeight;
	int m_iLandPlots;
	int m_iOwnedPlots;
	int m_iTopLatitude;
	int m_iBottomLatitude;
	int m_iNextRiverID;

	bool m_bWrapX;
	bool m_bWrapY;

	BonusArray<int> m_ja_NumBonuses;
	BonusArray<int> m_ja_NumBonusesOnLand;

	CvPlot* m_pMapPlots;

	FFreeListTrashArray<CvArea> m_areas;

	void calculateAreas();

};

/* <advc.make> Global wrappers for distance functions. */
inline int plotDistance(int iX1, int iY1, int iX2, int iY2) {
	return GC.getMap().plotDistance(iX1, iY1, iX2, iY2);
}
inline int plotDistance(const CvPlot* plot1, const CvPlot* plot2) {
	return GC.getMap().plotDistance(plot1, plot2);
}
inline int stepDistance(int iX1, int iY1, int iX2, int iY2) {
	return GC.getMap().stepDistance(iX1, iY1, iX2, iY2);
}
inline int stepDistance(const CvPlot* plot1, const CvPlot* plot2) {
	return GC.getMap().stepDistance(plot1, plot2);
}
inline CvPlot* plotDirection(int iX, int iY, DirectionTypes eDirection) {
	return GC.getMap().plotDirection(iX, iY, eDirection);
}
inline CvPlot* plotCardinalDirection(int iX, int iY, CardinalDirectionTypes eCardinalDirection) {
	return GC.getMap().plotCardinalDirection(iX, iY, eCardinalDirection);
}
inline CvPlot* plotXY(int iX, int iY, int iDX, int iDY) {
	return GC.getMap().plotXY(iX, iY, iDX, iDY);
}
inline CvPlot* plotXY(const CvPlot* pPlot, int iDX, int iDY) {
	return GC.getMap().plotXY(pPlot, iDX, iDY);
}
inline DirectionTypes directionXY(int iDX, int iDY) {
	return GC.getMap().directionXY(iDX, iDY);
}
inline DirectionTypes directionXY(const CvPlot* pFromPlot, const CvPlot* pToPlot) {
	return GC.getMap().directionXY(*pFromPlot, *pToPlot);
}
inline CvPlot* plotCity(int iX, int iY, CityPlotTypes ePlot) {
	return GC.getMap().plotCity(iX, iY, ePlot);
}
inline CityPlotTypes plotCityXY(int iCityX, int iCityY, CvPlot const& kPlot) {
	return GC.getMap().plotCityXY(iCityX, iCityY, kPlot);
}
inline bool adjacentOrSame(CvPlot const& kFirstPlot, CvPlot const& kSecondPlot) { // advc
	return GC.getMap().adjacentOrSame(kFirstPlot, kSecondPlot);
}
// </advc.make>
// WTP: Need a version that can deal with an index rather than CityPlotTypes until we overhaul the AI functions
inline CvPlot* plotCity(int iX, int iY, int idx) {
	return GC.getMap().plotCity(iX, iY, static_cast<CityPlotTypes>(idx));
}
inline CityPlotTypes plotCityXY(const CvCity* pCity, CvPlot const& kPlot) {
	return GC.getMap().plotCityXY(pCity->getX(), pCity->getY(), kPlot);
}

#endif
