// plot.cpp

#include "CvGameCoreDLL.h"
#include "CvPlot.h"
#include "CvCity.h"
#include "CvUnit.h"
#include "CvGlobals.h"
#include "CvArea.h"
#include "CvGameAI.h"
#include "CvDLLInterfaceIFaceBase.h"
#include "CvDLLSymbolIFaceBase.h"
#include "CvDLLEntityIFaceBase.h"
#include "CvDLLPlotBuilderIFaceBase.h"
#include "CvDLLEngineIFaceBase.h"
#include "CvDLLFlagEntityIFaceBase.h"
#include "CvMap.h"
#include "CvPlayerAI.h"
#include "CvTeamAI.h"
#include "CvGameCoreUtils.h"
#include "CvRandom.h"
#include "CvDLLFAStarIFaceBase.h"
#include "CvDLLEventReporterIFaceBase.h"
#include "CvInfos.h"
#include "FProfiler.h"
#include "CvArtFileMgr.h"
#include "CyArgsList.h"
#include "CvDLLPythonIFaceBase.h"
#include "CvGameTextMgr.h"

#define STANDARD_MINIMAP_ALPHA		(0.6f)


// Public Functions...

CvPlot::CvPlot()
{
	m_aiYield = new short[NUM_YIELD_TYPES];

	m_aiDangerMap = new short[MAX_PLAYERS];	// TAC - AI Improved Naval AI - koma13

	m_aiCulture = NULL;
	m_aiCultureRangeForts = NULL; // Super Forts *culture*
	m_aiFoundValue = NULL;
	m_aiPlayerCityRadiusCount = NULL;
	m_aiVisibilityCount = NULL;
	m_aiRevealedOwner = NULL;
	m_abRiverCrossing = NULL;
	m_abRevealed = NULL;
	m_aeRevealedImprovementType = NULL;
	m_aeRevealedRouteType = NULL;
	m_paiBuildProgress = NULL;
	m_apaiCultureRangeCities = NULL;
	m_apaiInvisibleVisibilityCount = NULL;

	m_pFeatureSymbol = NULL;
	m_pPlotBuilder = NULL;
	m_pRouteSymbol = NULL;
	m_pRiverSymbol = NULL;
	m_pFlagSymbol = NULL;
	m_pFlagSymbolOffset = NULL;
	m_pCenterUnit = NULL;

	m_szScriptData = NULL;

	reset(0, 0, true);
}


CvPlot::~CvPlot()
{
	uninit();

	SAFE_DELETE_ARRAY(m_aiYield);
	
	SAFE_DELETE_ARRAY(m_aiDangerMap);	// TAC - AI Improved Naval AI - koma13
}

void CvPlot::init(int iX, int iY)
{
	//--------------------------------
	// Init saved data
	reset(iX, iY);

	//--------------------------------
	// Init non-saved data

	//--------------------------------
	// Init other game data
}


void CvPlot::uninit()
{
	SAFE_DELETE_ARRAY(m_szScriptData);

	gDLL->getFeatureIFace()->destroy(m_pFeatureSymbol);
	gDLL->getPlotBuilderIFace()->destroy(m_pPlotBuilder);
	gDLL->getRouteIFace()->destroy(m_pRouteSymbol);
	gDLL->getRiverIFace()->destroy(m_pRiverSymbol);
	gDLL->getFlagEntityIFace()->destroy(m_pFlagSymbol);
	gDLL->getFlagEntityIFace()->destroy(m_pFlagSymbolOffset);
	m_pCenterUnit = NULL;

	SAFE_DELETE_ARRAY(m_aiCulture);
	SAFE_DELETE_ARRAY(m_aiCultureRangeForts); // Super Forts *culture*
	SAFE_DELETE_ARRAY(m_aiFoundValue);
	SAFE_DELETE_ARRAY(m_aiPlayerCityRadiusCount);

	SAFE_DELETE_ARRAY(m_aiVisibilityCount);
	SAFE_DELETE_ARRAY(m_aiRevealedOwner);

	SAFE_DELETE_ARRAY(m_abRiverCrossing);
	SAFE_DELETE_ARRAY(m_abRevealed);

	SAFE_DELETE_ARRAY(m_aeRevealedImprovementType);
	SAFE_DELETE_ARRAY(m_aeRevealedRouteType);

	SAFE_DELETE_ARRAY(m_paiBuildProgress);

	if (NULL != m_apaiCultureRangeCities)
	{
		for (int iI = 0; iI < MAX_PLAYERS; ++iI)
		{
			SAFE_DELETE_ARRAY(m_apaiCultureRangeCities[iI]);
		}
		SAFE_DELETE_ARRAY(m_apaiCultureRangeCities);
	}

	if (NULL != m_apaiInvisibleVisibilityCount)
	{
		for (int iI = 0; iI < MAX_TEAMS; ++iI)
		{
			SAFE_DELETE_ARRAY(m_apaiInvisibleVisibilityCount[iI]);
		}
		SAFE_DELETE_ARRAY(m_apaiInvisibleVisibilityCount);
	}

	m_units.clear();
}

// FUNCTION: reset()
// Initializes data members that are serialized.
void CvPlot::reset(int iX, int iY, bool bConstructorCall)
{
	uninit();

	m_iX = iX;
	m_iY = iY;
	m_iArea = FFreeList::INVALID_INDEX;
	m_pPlotArea = NULL;
	m_iFeatureVariety = 0;
	m_iOwnershipDuration = 0;
	m_iImprovementDuration = 0;
	m_iUpgradeProgress = 0;
	m_iForceUnownedTimer = 0;
	m_iCityRadiusCount = 0;
	m_iRiverID = -1;
	m_iMinOriginalStartDist = -1;
	m_iRiverCrossingCount = 0;
	m_iDistanceToOcean = MAX_SHORT;
	m_iCrumbs = 0;

	// Super Forts begin *canal* *choke*
	m_iCanalValue = 0;
	m_iChokeValue = 0;
	// Super Forts end
	// Super Forts begin *bombard*
	m_iDefenseDamage = 0;
	m_bBombarded = false;
	// Super Forts end

	m_bStartingPlot = false;
	m_bHills = false;
	m_bNOfRiver = false;
	m_bWOfRiver = false;
	m_bPotentialCityWork = false;
	m_bShowCitySymbols = false;
	m_bFlagDirty = false;
	m_bPlotLayoutDirty = false;
	m_bLayoutStateWorked = false;
	m_bImpassable = false;

	m_eOwner = NO_PLAYER;
	m_ePlotType = PLOT_OCEAN;
	m_eTerrainType = NO_TERRAIN;
	m_eFeatureType = NO_FEATURE;
	m_eBonusType = NO_BONUS;
	m_eImprovementType = NO_IMPROVEMENT;
	m_eRouteType = NO_ROUTE;
	m_eRiverNSDirection = NO_CARDINALDIRECTION;
	m_eRiverWEDirection = NO_CARDINALDIRECTION;
	m_eEurope = NO_EUROPE;

	m_plotCity.reset();
	m_workingCity.reset();
	m_workingCityOverride.reset();

	for (int iI = 0; iI < NUM_YIELD_TYPES; ++iI)
	{
		m_aiYield[iI] = 0;
	}

	// TAC - AI Improved Naval AI - koma13 - START
	for (int iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		m_aiDangerMap[iI] = 0;
	}
	// TAC - AI Improved Naval AI - koma13 - END
	
	updateImpassable();
}


//////////////////////////////////////
// graphical only setup
//////////////////////////////////////
void CvPlot::setupGraphical()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	updateSymbols();
	updateFeatureSymbol(false, false);
	updateRiverSymbol();
	updateMinimapColor();

	updateVisibility();
}

void CvPlot::erase()
{
	CLLNode<IDInfo>* pUnitNode;
	CvCity* pCity;
	CvUnit* pLoopUnit;
	CLinkList<IDInfo> oldUnits;

	// kill units
	oldUnits.clear();

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		oldUnits.insertAtEnd(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);
	}

	pUnitNode = oldUnits.head();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = oldUnits.next(pUnitNode);

		if (pLoopUnit != NULL)
		{
			pLoopUnit->kill(false);
		}
	}

	// kill cities
	pCity = getPlotCity();
	if (pCity != NULL)
	{
		pCity->kill();
	}

	setBonusType(NO_BONUS);
	setImprovementType(NO_IMPROVEMENT);
	setRouteType(NO_ROUTE);
	setFeatureType(NO_FEATURE);
	setEurope(NO_EUROPE);

	// disable rivers
	setNOfRiver(false, NO_CARDINALDIRECTION);
	setWOfRiver(false, NO_CARDINALDIRECTION);
	setRiverID(-1);
}


float CvPlot::getPointX() const
{
	return GC.getMapINLINE().plotXToPointX(getX_INLINE());
}


float CvPlot::getPointY() const
{
	return GC.getMapINLINE().plotYToPointY(getY_INLINE());
}


NiPoint3 CvPlot::getPoint() const
{
	NiPoint3 pt3Point;

	pt3Point.x = getPointX();
	pt3Point.y = getPointY();
	pt3Point.z = 0.0f;

	pt3Point.z = gDLL->getEngineIFace()->GetHeightmapZ(pt3Point);

	return pt3Point;
}

TeamTypes CvPlot::getTeam() const
{
	if (isOwned())
	{
		return GET_PLAYER(getOwnerINLINE()).getTeam();
	}
	else
	{
		return NO_TEAM;
	}
}


void CvPlot::doTurn()
{
	PROFILE_FUNC();

	if (getForceUnownedTimer() > 0)
	{
		changeForceUnownedTimer(-1);
	}

	if (isOwned())
	{
		changeOwnershipDuration(1);
	}

	if (getImprovementType() != NO_IMPROVEMENT && getOwnerINLINE() != NO_PLAYER)
	{
		changeImprovementDuration(1);
		doImprovementUpgrade();			
	}

	// Super Forts begin *bombard*
	if (!isBombarded() && getDefenseDamage() > 0)
	{
		changeDefenseDamage(-(GC.getDefineINT("CITY_DEFENSE_DAMAGE_HEAL_RATE")));
	}
	setBombarded(false);
	// Super Forts end
	doFeature();

	//R&R mod, vetiarvind, super forts merge
	//updateCulture(true); -- original
	doCulture();
	//super forts end

	verifyUnitValidPlot();
	
	
	// R&R, ray, Monasteries and Forts - START	
	doFort();
	doMonastery();
	// R&R, ray, Monasteries and Forts - END

	// XXX
#ifdef _DEBUG
	{
		CLLNode<IDInfo>* pUnitNode;
		CvUnit* pLoopUnit;

		pUnitNode = headUnitNode();

		while (pUnitNode != NULL)
		{
			pLoopUnit = ::getUnit(pUnitNode->m_data);
			pUnitNode = nextUnitNode(pUnitNode);

			FAssertMsg(pLoopUnit->atPlot(this), "pLoopUnit is expected to be at the current plot instance");
		}
	}
#endif
	// XXX

	m_iCrumbs = (m_iCrumbs * 95) / 100;
}


void CvPlot::doImprovement()
{
	PROFILE_FUNC();

	CvCity* pCity;
	CvWString szBuffer;
	int iI;

	FAssert(isBeingWorked() && isOwned());

	if (getImprovementType() != NO_IMPROVEMENT)
	{
		if (getBonusType() == NO_BONUS)
		{
			FAssertMsg((0 < GC.getNumBonusInfos()), "GC.getNumBonusInfos() is not greater than zero but an array is being allocated in CvPlot::doImprovement");
			for (iI = 0; iI < GC.getNumBonusInfos(); ++iI)
			{
				if (GC.getImprovementInfo(getImprovementType()).getImprovementBonusDiscoverRand(iI) > 0)
				{
					if (GC.getGameINLINE().getSorenRandNum(GC.getImprovementInfo(getImprovementType()).getImprovementBonusDiscoverRand(iI), "Bonus Discovery") == 0)
					{
						setBonusType((BonusTypes)iI);
						pCity = GC.getMapINLINE().findCity(getX_INLINE(), getY_INLINE(), getOwnerINLINE(), NO_TEAM, false);
						if (pCity != NULL)
						{
							szBuffer = gDLL->getText("TXT_KEY_MISC_DISCOVERED_NEW_RESOURCE", GC.getBonusInfo((BonusTypes) iI).getTextKeyWide(), pCity->getNameKey());
							gDLL->getInterfaceIFace()->addMessage(getOwnerINLINE(), false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_DISCOVERBONUS", MESSAGE_TYPE_MINOR_EVENT, GC.getBonusInfo((BonusTypes) iI).getButton(), (ColorTypes)GC.getInfoTypeForString("COLOR_WHITE"), getX_INLINE(), getY_INLINE(), true, true);
						}
						break;
					}
				}
			}
		}
	}
}

void CvPlot::upgradeImprovement(ImprovementTypes eImprovementUpgrade, int iUpgradeRate)
{
	changeUpgradeProgress(iUpgradeRate);

	if (getUpgradeProgress() >= GC.getGameINLINE().getImprovementUpgradeTime(getImprovementType()))
	{
		setImprovementType(eImprovementUpgrade);
	}

}

void CvPlot::doImprovementUpgrade()
{
	FAssert(getImprovementType() != NO_IMPROVEMENT);

	const ImprovementTypes eImprovementUpgrade = (ImprovementTypes)GC.getImprovementInfo(getImprovementType()).getImprovementUpgrade();

	if (eImprovementUpgrade != NO_IMPROVEMENT)
	{
		const CvImprovementInfo& info = GC.getImprovementInfo(eImprovementUpgrade);

		const bool bIsOutsideBorder = info.isOutsideBorders();

		const int iUpgradeRate = GET_PLAYER(getOwnerINLINE()).getImprovementUpgradeRate();

		// Erik: Improvements that are allowd outside borders require specific units to activate them with regards to growth.
		if (bIsOutsideBorder)
		{
			// Erik: Note that this case also handles forts \ monasteries that are worked inside a city AND is activated
			if (isFort() || isMonastery())
			{
				const bool bDefenderFound = (getFortDefender() != NULL);
				const bool bMissionaryFound = (getMonasteryMissionary() != NULL);

				if (bDefenderFound || bMissionaryFound)
				{
					upgradeImprovement(eImprovementUpgrade, iUpgradeRate);
				}
			}
			else
			{
				// Erik: Currently unused by RaR, but a future improvement may support upgrades so we allow that unconditionally as long as its not a fort or monasteryy
				upgradeImprovement(eImprovementUpgrade, iUpgradeRate);
			}
		}

		// Erik: This is the standard case for farms etc.
		if (isBeingWorked() && !bIsOutsideBorder)
		{
			upgradeImprovement(eImprovementUpgrade, iUpgradeRate);
		}
	}
}

void CvPlot::updateCulture(bool bBumpUnits)
{
	// Super Forts begin *culture*
	if (!isCity(true) || (getOwnerINLINE() == NO_PLAYER))
	// if (!isCity()) Original Code
	// Super Forts end
	{
		setOwner(calculateCulturalOwner(), bBumpUnits);
	}
}


void CvPlot::updateFog()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	FAssert(GC.getGameINLINE().getActiveTeam() != NO_TEAM);

	if (isRevealed(GC.getGameINLINE().getActiveTeam(), false))
	{
		if (gDLL->getInterfaceIFace()->isBareMapMode())
		{
			gDLL->getEngineIFace()->LightenVisibility(getFOWIndex());
		}
		else
		{
			int cityScreenFogEnabled = GC.getDefineINT("CITY_SCREEN_FOG_ENABLED");
			if (cityScreenFogEnabled && gDLL->getInterfaceIFace()->isCityScreenUp() && (gDLL->getInterfaceIFace()->getHeadSelectedCity() != getWorkingCity()))
			{
				gDLL->getEngineIFace()->DarkenVisibility(getFOWIndex());
			}
			else if (isActiveVisible(false))
			{
				gDLL->getEngineIFace()->LightenVisibility(getFOWIndex());
			}
			else
			{
				gDLL->getEngineIFace()->DarkenVisibility(getFOWIndex());
			}
		}
	}
	else
	{
		gDLL->getEngineIFace()->BlackenVisibility(getFOWIndex());
	}
}


void CvPlot::updateVisibility()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	setLayoutDirty(true);

	updateSymbolVisibility();
	updateFeatureSymbolVisibility();
	updateRouteSymbol();

	CvCity* pCity = getPlotCity();
	if (pCity != NULL)
	{
		pCity->updateVisibility();
	}
}


void CvPlot::updateSymbolDisplay()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	//figure out which yield to highlight
	// R&R, ray , MYCP partially based on code of Aymerick - START
	std::set<YieldTypes> setYieldsProduced;
	CvCity *pPlotCity = NULL;
	if (isVisibleWorked())
	{
		pPlotCity = getPlotCity();
		CvCity* pWorkingCity = getWorkingCity();
		if (pWorkingCity != NULL)
		{
			const CvUnit* pWorkingUnit = pWorkingCity->getUnitWorkingPlot(this);
			if (pWorkingUnit != NULL)
			{
				ProfessionTypes eProfession = pWorkingUnit->getProfession();
				if (NO_PROFESSION != eProfession)
				{
					CvProfessionInfo& kProfession = GC.getProfessionInfo(eProfession);
					// R&R, ray, adjustment, we only highlight first yield on plot
					/* for (int iYieldProduced = 0; iYieldProduced < kProfession.getNumYieldsProduced(); iYieldProduced++)
					{
						if (kProfession.isWater() == isWater())
						{
							setYieldsProduced.insert((YieldTypes)kProfession.getYieldsProduced(iYieldProduced));
						}
					}
					*/
					if (kProfession.isWater() == isWater())
					{
						setYieldsProduced.insert((YieldTypes)kProfession.getYieldsProduced(0));
					}
					// R&R, ray, adjustment, END
				}
			}
		}

	}
	// R&R, ray , MYCP partially based on code of Aymerick - END
	if(isShowCitySymbols() && (getPlotCity() == NULL))
	{
		gDLL->getEngineIFace()->setYieldSymbolOffset(this, 0.2f * GC.getPLOT_SIZE());
	}
	else
	{
		gDLL->getEngineIFace()->setYieldSymbolOffset(this, 0);
	}
	// R&R, ray , MYCP partially based on code of Aymerick - START
	for (int i = 0; i < GC.getNUM_YIELD_TYPES(); i++)
	{
		YieldTypes eYield = (YieldTypes) i;
		if (isShowCitySymbols())
		{
			if ((setYieldsProduced.find(eYield) != setYieldsProduced.end()) || (pPlotCity != NULL))
			{
				gDLL->getEngineIFace()->setYieldSymbolAppearance(this, eYield, 1.0f, 1.6f, true);
			}
			else
			{
				gDLL->getEngineIFace()->setYieldSymbolAppearance(this, eYield, 0.8f, 1.2f, false);
			}
		}
		else
		{
			gDLL->getEngineIFace()->setYieldSymbolAppearance(this, eYield, 0.6f, 0.8f, false);
		}
	}
	// R&R, ray , MYCP partially based on code of Aymerick - END
}

void CvPlot::updateSymbolVisibility()
{
	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	if (isRevealed(GC.getGameINLINE().getActiveTeam(), true) &&
			(isShowCitySymbols() ||
			(gDLL->getInterfaceIFace()->isShowYields() && !(gDLL->getInterfaceIFace()->isCityScreenUp()))))
	{
		gDLL->getEngineIFace()->setYieldSymbolVisible(this, true);
	}
	else
	{
		gDLL->getEngineIFace()->setYieldSymbolVisible(this, false);
	}
}


void CvPlot::updateSymbols()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	gDLL->getEngineIFace()->clearYieldSymbol(this);

	for (int iYieldType = 0; iYieldType < NUM_YIELD_TYPES; iYieldType++)
	{
		YieldTypes eYield = (YieldTypes) iYieldType;
		if(GC.getYieldInfo(eYield).isCargo())
		{
			int iYield;
			if (isShowCitySymbols())
			{
				iYield = calculateYield(eYield, true);
			}
			else
			{
				iYield = calculatePotentialYield(eYield, NULL, true);
			}
			if(iYield > 0)
			{
				gDLL->getEngineIFace()->setYieldSymbolYieldAmount(this, eYield, iYield);
			}
		}
	}

	updateSymbolDisplay();
	updateSymbolVisibility();
}


void CvPlot::updateMinimapColor()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	gDLL->getInterfaceIFace()->setMinimapColor(MINIMAPMODE_TERRITORY, getX_INLINE(), getY_INLINE(), plotMinimapColor(), STANDARD_MINIMAP_ALPHA);
}


void CvPlot::updateCenterUnit()
{
	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	if (!isActiveVisible(true))
	{
		setCenterUnit(NULL);
		return;
	}

	setCenterUnit(getSelectedUnit());

	if (getCenterUnit() == NULL)
	{
		setCenterUnit(getBestDefender(GC.getGameINLINE().getActivePlayer(), NO_PLAYER, NULL, false, false, true));
	}

	if (getCenterUnit() == NULL)
	{
		setCenterUnit(getBestDefender(GC.getGameINLINE().getActivePlayer()));
	}

	if (getCenterUnit() == NULL)
	{
		setCenterUnit(getBestDefender(NO_PLAYER, GC.getGameINLINE().getActivePlayer(), gDLL->getInterfaceIFace()->getHeadSelectedUnit(), true));
	}

	if (getCenterUnit() == NULL)
	{
		setCenterUnit(getBestDefender(NO_PLAYER, GC.getGameINLINE().getActivePlayer(), gDLL->getInterfaceIFace()->getHeadSelectedUnit()));
	}

	if (getCenterUnit() == NULL)
	{
		setCenterUnit(getBestDefender(NO_PLAYER, GC.getGameINLINE().getActivePlayer()));
	}
}


void CvPlot::verifyUnitValidPlot()
{
	std::vector<CvUnit*> aUnits;
	CLLNode<IDInfo>* pUnitNode = headUnitNode();
	while (pUnitNode != NULL)
	{
		CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);
		if (NULL != pLoopUnit)
		{
			aUnits.push_back(pLoopUnit);
		}
	}


	for (std::vector<CvUnit*>::iterator it = aUnits.begin(); it != aUnits.end(); ++it)
	{
		CvUnit* pLoopUnit = *it;
		if (pLoopUnit->atPlot(this))
		{
			if (!(pLoopUnit->isCargo()))
			{
				if (!(pLoopUnit->isCombat()))
				{
					if (!pLoopUnit->isValidPlot(this))
					{
						pLoopUnit->jumpToNearestValidPlot();
					}
				}
			}
		}
	}
}

bool CvPlot::isAdjacentToPlot(CvPlot* pPlot) const
{
    return (stepDistance(getX_INLINE(), getY_INLINE(), pPlot->getX_INLINE(), pPlot->getY_INLINE()) == 1);
}

bool CvPlot::isAdjacentToArea(int iAreaID) const
{
	PROFILE_FUNC();

	for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		CvPlot* pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->getArea() == iAreaID)
			{
				return true;
			}
		}
	}

	return false;
}

bool CvPlot::isAdjacentToArea(const CvArea* pArea) const
{
	return isAdjacentToArea(pArea->getID());
}


bool CvPlot::shareAdjacentArea(const CvPlot* pPlot) const
{
	PROFILE_FUNC();

	int iLastArea = FFreeList::INVALID_INDEX;

	for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		CvPlot* pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			int iCurrArea = pAdjacentPlot->getArea();

			if (iCurrArea != iLastArea)
			{
				if (pPlot->isAdjacentToArea(iCurrArea))
				{
					return true;
				}

				iLastArea = iCurrArea;
			}
		}
	}

	return false;
}


bool CvPlot::isAdjacentToLand() const
{
	PROFILE_FUNC();

	for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		CvPlot* pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (!(pAdjacentPlot->isWater()))
			{
				return true;
			}
		}
	}

	return false;
}


bool CvPlot::isCoastalLand(int iMinWaterSize) const
{
	PROFILE_FUNC();

	if (isWater())
	{
		return false;
	}

	for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		CvPlot* pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->isWater())
			{
				if (pAdjacentPlot->area()->getNumTiles() >= iMinWaterSize)
				{
					return true;
				}
			}
		}
	}

	return false;
}

bool CvPlot::isAdjacentWaterPassable(CvPlot* pPlot) const
{
	FAssert(pPlot != NULL);
	FAssert(isAdjacentToPlot(pPlot));
	FAssert(isWater() && pPlot->isWater());
	
	DirectionTypes eDirection = directionXY(this, pPlot);
	
	if (isCardinalDirection(eDirection))
	{
		return true;
	}

	CvPlot* pDirectionPlot = plotDirection(pPlot->getX_INLINE(), pPlot->getY_INLINE(), (DirectionTypes)((eDirection + 1) % NUM_DIRECTION_TYPES));
	if (pDirectionPlot != NULL)
	{
		if (pDirectionPlot->isWater())
		{
			return true;
		}
	}
	pDirectionPlot = plotDirection(pPlot->getX_INLINE(), pPlot->getY_INLINE(), (DirectionTypes)((eDirection - 1) % NUM_DIRECTION_TYPES));
	if (pDirectionPlot != NULL)
	{
		if (pDirectionPlot->isWater())
		{
			return true;
		}
	}
	return false;
}

bool CvPlot::isVisibleWorked() const
{
	if (isBeingWorked())
	{
		if ((getTeam() == GC.getGameINLINE().getActiveTeam()) || GC.getGameINLINE().isDebugMode())
		{
			return true;
		}
	}

	return false;
}


bool CvPlot::isWithinTeamCityRadius(TeamTypes eTeam, PlayerTypes eIgnorePlayer) const
{
	PROFILE_FUNC();

	for (int iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		if (GET_PLAYER((PlayerTypes)iI).isAlive())
		{
			if (GET_PLAYER((PlayerTypes)iI).getTeam() == eTeam)
			{
				if ((eIgnorePlayer == NO_PLAYER) || (((PlayerTypes)iI) != eIgnorePlayer))
				{
					if (isPlayerCityRadius((PlayerTypes)iI))
					{
						return true;
					}
				}
			}
		}
	}

	return false;
}


bool CvPlot::isLake() const
{
	CvArea* pArea;

	pArea = area();

	if (pArea != NULL)
	{
		return pArea->isLake();
	}

	return false;
}

bool CvPlot::isRiverMask() const
{
	CvPlot* pPlot;

	if (isNOfRiver())
	{
		return true;
	}

	if (isWOfRiver())
	{
		return true;
	}

	pPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_EAST);
	if ((pPlot != NULL) && pPlot->isNOfRiver())
	{
		return true;
	}

	pPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_SOUTH);
	if ((pPlot != NULL) && pPlot->isWOfRiver())
	{
		return true;
	}

	return false;
}


bool CvPlot::isRiverCrossingFlowClockwise(DirectionTypes eDirection) const
{
	CvPlot *pPlot;
	switch(eDirection)
	{
	case DIRECTION_NORTH:
		pPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_NORTH);
		if (pPlot != NULL)
		{
			return (pPlot->getRiverWEDirection() == CARDINALDIRECTION_EAST);
		}
		break;
	case DIRECTION_EAST:
		return (getRiverNSDirection() == CARDINALDIRECTION_SOUTH);
		break;
	case DIRECTION_SOUTH:
		return (getRiverWEDirection() == CARDINALDIRECTION_WEST);
		break;
	case DIRECTION_WEST:
		pPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_WEST);
		if(pPlot != NULL)
		{
			return (pPlot->getRiverNSDirection() == CARDINALDIRECTION_NORTH);
		}
		break;
	default:
		FAssert(false);
		break;
	}

	return false;
}


bool CvPlot::isRiverSide() const
{
	CvPlot* pLoopPlot;
	int iI;

	for (iI = 0; iI < NUM_CARDINALDIRECTION_TYPES; ++iI)
	{
		pLoopPlot = plotCardinalDirection(getX_INLINE(), getY_INLINE(), ((CardinalDirectionTypes)iI));

		if (pLoopPlot != NULL)
		{
			if (isRiverCrossing(directionXY(this, pLoopPlot)))
			{
				return true;
			}
		}
	}

	return false;
}


bool CvPlot::isRiver() const
{
	return (getRiverCrossingCount() > 0);
}


bool CvPlot::isRiverConnection(DirectionTypes eDirection) const
{
	if (eDirection == NO_DIRECTION)
	{
		return false;
	}

	switch (eDirection)
	{
	case DIRECTION_NORTH:
		return (isRiverCrossing(DIRECTION_EAST) || isRiverCrossing(DIRECTION_WEST));
		break;

	case DIRECTION_NORTHEAST:
		return (isRiverCrossing(DIRECTION_NORTH) || isRiverCrossing(DIRECTION_EAST));
		break;

	case DIRECTION_EAST:
		return (isRiverCrossing(DIRECTION_NORTH) || isRiverCrossing(DIRECTION_SOUTH));
		break;

	case DIRECTION_SOUTHEAST:
		return (isRiverCrossing(DIRECTION_SOUTH) || isRiverCrossing(DIRECTION_EAST));
		break;

	case DIRECTION_SOUTH:
		return (isRiverCrossing(DIRECTION_EAST) || isRiverCrossing(DIRECTION_WEST));
		break;

	case DIRECTION_SOUTHWEST:
		return (isRiverCrossing(DIRECTION_SOUTH) || isRiverCrossing(DIRECTION_WEST));
		break;

	case DIRECTION_WEST:
		return (isRiverCrossing(DIRECTION_NORTH) || isRiverCrossing(DIRECTION_SOUTH));
		break;

	case DIRECTION_NORTHWEST:
		return (isRiverCrossing(DIRECTION_NORTH) || isRiverCrossing(DIRECTION_WEST));
		break;

	default:
		FAssert(false);
		break;
	}

	return false;
}


CvPlot* CvPlot::getNearestLandPlotInternal(int iDistance) const
{
	if (iDistance > GC.getMapINLINE().getGridHeightINLINE() && iDistance > GC.getMapINLINE().getGridWidthINLINE())
	{
		return NULL;
	}

	for (int iDX = -iDistance; iDX <= iDistance; iDX++)
	{
		for (int iDY = -iDistance; iDY <= iDistance; iDY++)
		{
			if (abs(iDX) + abs(iDY) == iDistance)
			{
				CvPlot* pPlot = plotXY(getX_INLINE(), getY_INLINE(), iDX, iDY);
				if (pPlot != NULL)
				{
					if (!pPlot->isWater())
					{
						return pPlot;
					}
				}
			}
		}
	}
	return getNearestLandPlotInternal(iDistance + 1);
}


int CvPlot::getNearestLandArea() const
{
	CvPlot* pPlot = getNearestLandPlot();
	return pPlot ? pPlot->getArea() : -1;
}


CvPlot* CvPlot::getNearestLandPlot() const
{
	return getNearestLandPlotInternal(0);
}


int CvPlot::seeFromLevel(TeamTypes eTeam) const
{
	int iLevel;

	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	iLevel = GC.getTerrainInfo(getTerrainType()).getSeeFromLevel();

	// Super Forts begin *vision*
	if (getImprovementType() != NO_IMPROVEMENT)
	{
		iLevel += GC.getImprovementInfo(getImprovementType()).getSeeFrom();
	}
	// Super Forts end

	if (isPeak())
	{
		iLevel += GC.getPEAK_SEE_FROM_CHANGE();
	}

	if (isHills())
	{
		iLevel += GC.getHILLS_SEE_FROM_CHANGE();
	}

	if (isWater())
	{
		iLevel += GC.getSEAWATER_SEE_FROM_CHANGE();
	}

	return iLevel;
}


int CvPlot::seeThroughLevel() const
{
	int iLevel;

	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	iLevel = GC.getTerrainInfo(getTerrainType()).getSeeThroughLevel();

	if (getFeatureType() != NO_FEATURE)
	{
		iLevel += GC.getFeatureInfo(getFeatureType()).getSeeThroughChange();
	}

	if (isPeak())
	{
		iLevel += GC.getPEAK_SEE_THROUGH_CHANGE();
	}

	if (isHills())
	{
		iLevel += GC.getHILLS_SEE_THROUGH_CHANGE();
	}

	if (isWater())
	{
		iLevel += GC.getSEAWATER_SEE_FROM_CHANGE();
	}

	return iLevel;
}



void CvPlot::changeAdjacentSight(TeamTypes eTeam, int iRange, bool bIncrement, CvUnit* pUnit)
{
	if(pUnit != NULL)
	{
		if(!pUnit->isOnMap())
		{
			return;
		}
	}

	DirectionTypes eFacingDirection = NO_DIRECTION;

	if (NULL != pUnit)
	{
		eFacingDirection = pUnit->getFacingDirection(true);
	}

	//fill invisible types
	std::vector<InvisibleTypes> aSeeInvisibleTypes;
	if (NULL != pUnit)
	{
		for(int i=0;i<pUnit->getNumSeeInvisibleTypes();i++)
		{
			aSeeInvisibleTypes.push_back(pUnit->getSeeInvisibleType(i));
		}
	}

	if(aSeeInvisibleTypes.size() == 0)
	{
		aSeeInvisibleTypes.push_back(NO_INVISIBLE);
	}

	//check one extra outer ring
	iRange++;

	for(int i=0;i<(int)aSeeInvisibleTypes.size();i++)
	{
		for (int dx = -iRange; dx <= iRange; dx++)
		{
			for (int dy = -iRange; dy <= iRange; dy++)
			{
				//check if in facing direction
				if (shouldProcessDisplacementPlot(dx, dy, iRange - 1, eFacingDirection))
				{
					bool outerRing = false;
					if ((abs(dx) == iRange) || (abs(dy) == iRange))
					{
						outerRing = true;
					}

					//check if anything blocking the plot
					if (canSeeDisplacementPlot(eTeam, dx, dy, dx, dy, true, outerRing))
					{
						CvPlot* pPlot = plotXY(getX_INLINE(), getY_INLINE(), dx, dy);
						if (NULL != pPlot)
						{
							pPlot->changeVisibilityCount(eTeam, ((bIncrement) ? 1 : -1), aSeeInvisibleTypes[i]);
						}
					}
				}

				if (eFacingDirection != NO_DIRECTION)
				{
					if((abs(dx) <= 1) && (abs(dy) <= 1)) //always reveal adjacent plots when using line of sight
					{
						CvPlot* pPlot = plotXY(getX_INLINE(), getY_INLINE(), dx, dy);
						if (NULL != pPlot)
						{
							pPlot->changeVisibilityCount(eTeam, 1, aSeeInvisibleTypes[i]);
							pPlot->changeVisibilityCount(eTeam, -1, aSeeInvisibleTypes[i]);
						}
					}
				}
			}
		}
	}
}

bool CvPlot::canSeePlot(CvPlot *pPlot, TeamTypes eTeam, int iRange, DirectionTypes eFacingDirection) const
{
	iRange++;

	if (pPlot == NULL)
	{
		return false;
	}

	//find displacement
	int dx = pPlot->getX() - getX();
	int dy = pPlot->getY() - getY();
	dx = dxWrap(dx); //world wrap
	dy = dyWrap(dy);

	//check if in facing direction
	if (shouldProcessDisplacementPlot(dx, dy, iRange - 1, eFacingDirection))
	{
		bool outerRing = false;
		if ((abs(dx) == iRange) || (abs(dy) == iRange))
		{
			outerRing = true;
		}

		//check if anything blocking the plot
		if (canSeeDisplacementPlot(eTeam, dx, dy, dx, dy, true, outerRing))
		{
			return true;
		}
	}

	return false;
}

bool CvPlot::canSeeDisplacementPlot(TeamTypes eTeam, int dx, int dy, int originalDX, int originalDY, bool firstPlot, bool outerRing) const
{
	CvPlot *pPlot = plotXY(getX_INLINE(), getY_INLINE(), dx, dy);
	if (pPlot != NULL)
	{
		//base case is current plot
		if((dx == 0) && (dy == 0))
		{
			return true;
		}

		//find closest of three points (1, 2, 3) to original line from Start (S) to End (E)
		//The diagonal is computed first as that guarantees a change in position
		// -------------
		// |   | 2 | S |
		// -------------
		// | E | 1 | 3 |
		// -------------

		int displacements[3][2] = {{dx - getSign(dx), dy - getSign(dy)}, {dx - getSign(dx), dy}, {dx, dy - getSign(dy)}};
		int allClosest[3];
		int closest = -1;
		for (int i=0;i<3;i++)
		{
			//int tempClosest = abs(displacements[i][0] * originalDX - displacements[i][1] * originalDY); //more accurate, but less structured on a grid
			allClosest[i] = abs(displacements[i][0] * dy - displacements[i][1] * dx); //cross product
			if((closest == -1) || (allClosest[i] < closest))
			{
				closest = allClosest[i];
			}
		}

		//iterate through all minimum plots to see if any of them are passable
		for(int i=0;i<3;i++)
		{
			int nextDX = displacements[i][0];
			int nextDY = displacements[i][1];
			if((nextDX != dx) || (nextDY != dy)) //make sure we change plots
			{
				if(allClosest[i] == closest)
				{
					if(canSeeDisplacementPlot(eTeam, nextDX, nextDY, originalDX, originalDY, false, false))
					{
						int fromLevel = seeFromLevel(eTeam);
						int throughLevel = pPlot->seeThroughLevel();
						if(outerRing) //check strictly higher level
						{
							CvPlot *passThroughPlot = plotXY(getX_INLINE(), getY_INLINE(), nextDX, nextDY);
							int passThroughLevel = passThroughPlot->seeThroughLevel();
							if (fromLevel >= passThroughLevel)
							{
								if((fromLevel > passThroughLevel) || (pPlot->seeFromLevel(eTeam) > fromLevel)) //either we can see through to it or it is high enough to see from far
								{
									return true;
								}
							}
						}
						else
						{
							if(fromLevel >= throughLevel) //we can clearly see this level
							{
								return true;
							}
							else if(firstPlot) //we can also see it if it is the first plot that is too tall
							{
								return true;
							}
						}
					}
				}
			}
		}
	}

	return false;
}

bool CvPlot::shouldProcessDisplacementPlot(int dx, int dy, int range, DirectionTypes eFacingDirection) const
{
	if(eFacingDirection == NO_DIRECTION)
	{
		return true;
	}
	else if((dx == 0) && (dy == 0)) //always process this plot
	{
		return true;
	}
	else
	{
		//							N		NE		E		SE			S		SW		W			NW
		int displacements[8][2] = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

		int directionX = displacements[eFacingDirection][0];
		int directionY = displacements[eFacingDirection][1];

		//compute angle off of direction
		int crossProduct = directionX * dy - directionY * dx; //cross product
		int dotProduct = directionX * dx + directionY * dy; //dot product

		float theta = atan2((float) crossProduct, (float) dotProduct);
		float spread = 60 * (float) M_PI / 180;
		if((abs(dx) <= 1) && (abs(dy) <= 1)) //close plots use wider spread
		{
			spread = 90 * (float) M_PI / 180;
		}

		if((theta >= -spread / 2) && (theta <= spread / 2))
		{
			return true;
		}
		else
		{
			return false;
		}

		/*
		DirectionTypes leftDirection = GC.getTurnLeftDirection(eFacingDirection);
		DirectionTypes rightDirection = GC.getTurnRightDirection(eFacingDirection);

		//test which sides of the line equation (cross product)
		int leftSide = displacements[leftDirection][0] * dy - displacements[leftDirection][1] * dx;
		int rightSide = displacements[rightDirection][0] * dy - displacements[rightDirection][1] * dx;
		if((leftSide <= 0) && (rightSide >= 0))
			return true;
		else
			return false;
		*/
	}
}

void CvPlot::updateSight(bool bIncrement)
{
	CvCity* pCity = getPlotCity();

	// Owned
	if (isOwned())
	{
		changeAdjacentSight(getTeam(), GC.getPLOT_VISIBILITY_RANGE(), bIncrement, NULL);
	}

	// Unit
	for (CLLNode<IDInfo>* pUnitNode = headUnitNode(); pUnitNode != NULL; pUnitNode = nextUnitNode(pUnitNode))
	{
		CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
		if (pLoopUnit->isOnMap())
		{
			changeAdjacentSight(pLoopUnit->getTeam(), pLoopUnit->visibilityRange(), bIncrement, pLoopUnit);
		}
	}
}


void CvPlot::updateSeeFromSight(bool bIncrement)
{
	CvPlot* pLoopPlot;
	int iDX, iDY;

	int iRange = GC.getUNIT_VISIBILITY_RANGE() + 1;
	for (int iPromotion = 0; iPromotion < GC.getNumPromotionInfos(); ++iPromotion)
	{
		iRange += GC.getPromotionInfo((PromotionTypes)iPromotion).getVisibilityChange();
	}

	for (iDX = -iRange; iDX <= iRange; iDX++)
	{
		for (iDY = -iRange; iDY <= iRange; iDY++)
		{
			pLoopPlot = plotXY(getX_INLINE(), getY_INLINE(), iDX, iDY);

			if (pLoopPlot != NULL)
			{
				pLoopPlot->updateSight(bIncrement);
			}
		}
	}
}


bool CvPlot::canHaveBonus(BonusTypes eBonus, bool bIgnoreLatitude) const
{
	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	if (eBonus == NO_BONUS)
	{
		return true;
	}

	if (getBonusType() != NO_BONUS)
	{
		return false;
	}

	if (isPeak())
	{
		// R&R, ray, Bonus on Peaks - START
		//return false;
		if (!(GC.getBonusInfo(eBonus).isPeaks()))
		{
			return false;
		}
		// R&R, ray, Bonus on Peaks - END
	}

	if (getFeatureType() != NO_FEATURE)
	{
		if (!(GC.getBonusInfo(eBonus).isFeature(getFeatureType())))
		{
			return false;
		}

		if (!(GC.getBonusInfo(eBonus).isFeatureTerrain(getTerrainType())))
		{
			return false;
		}
	}
	else
	{
		if (!(GC.getBonusInfo(eBonus).isTerrain(getTerrainType())))
		{
			return false;
		}
	}
	
	// R&R, ray, Bonus on Peaks - START
	//if (isHills() || isPeak())
	if (isHills())
	// R&R, ray, Bonus on Peaks - END
	{
		if (!(GC.getBonusInfo(eBonus).isHills()))
		{
			return false;
		}
	}
	else if (isFlatlands())
	{
		if (!(GC.getBonusInfo(eBonus).isFlatlands()))
		{
			return false;
		}
	}

	if (GC.getBonusInfo(eBonus).isNoRiverSide())
	{
		if (isRiverSide())
		{
			return false;
		}
	}

	if (GC.getBonusInfo(eBonus).getMinAreaSize() != -1)
	{
		if (area()->getNumTiles() < GC.getBonusInfo(eBonus).getMinAreaSize())
		{
			return false;
		}
	}

	if (!bIgnoreLatitude)
	{
		if (getLatitude() > GC.getBonusInfo(eBonus).getMaxLatitude())
		{
			return false;
		}

		if (getLatitude() < GC.getBonusInfo(eBonus).getMinLatitude())
		{
			return false;
		}
	}

	//TAC Whaling, ray
	if (isWater() && !GC.getBonusInfo(eBonus).isOcean())
	{
		if (!isPotentialCityWork())
		{
			return false;
		}
	}

	/*if (!isPotentialCityWork())
	{
		return false;
	}*/
	//End TAC Whaling, ray

	return true;
}


bool CvPlot::canHaveImprovement(ImprovementTypes eImprovement, TeamTypes eTeam, bool bPotential, /* build feature removal detection - Nightinggale */ bool bIgnoreFeature) const
{
	CvPlot* pLoopPlot;
	bool bValid;
	int iI;

	FAssertMsg(eImprovement != NO_IMPROVEMENT, "Improvement is not assigned a valid value");
	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	bValid = false;

	if (isCity())
	{
		return false;
	}

	if (isImpassable())
	{
		return false;
	}

	if (GC.getImprovementInfo(eImprovement).isWater() != isWater())
	{
		return false;
	}

	if (getFeatureType() != NO_FEATURE)
	{
		if (GC.getFeatureInfo(getFeatureType()).isNoImprovement())
		{
			return false;
		}
	}

	// R&R, ray, Improving AI logic for Improvements
	if (eImprovement != NO_IMPROVEMENT && getBonusType() != NO_BONUS && eTeam != NO_TEAM && !GET_TEAM(eTeam).isHuman())
	{
		bool improvementDoesMatchBonus = false;
		CvBonusInfo& kBonus = GC.getBonusInfo(getBonusType());
		CvImprovementInfo& kImprovement = GC.getImprovementInfo(eImprovement);

		for (int xx = 0; xx < NUM_YIELD_TYPES && !improvementDoesMatchBonus; ++xx)
		{
			if(kBonus.getYieldChange(xx) > 0 && kImprovement.getYieldIncrease(xx) > 0)
			{
				improvementDoesMatchBonus = true;
			}
		}

		if (improvementDoesMatchBonus == false)
		{
			return false;
		}
	}

	// R&R, ray, Livestock Breeding, for AI
	CvCity* pWorkingCity = getWorkingCity();
	if (pWorkingCity != NULL && eTeam != NO_TEAM && !GET_TEAM(eTeam).isHuman()  && getBonusType() == NO_BONUS) 
	{
		CvImprovementInfo& kImprovement = GC.getImprovementInfo(eImprovement);
		bool isLivestockImprovment = false;

		for (int yy = 0; yy < NUM_YIELD_TYPES && !isLivestockImprovment; ++yy)
		{
			if (GC.getYieldInfo((YieldTypes) yy).isLivestock() && kImprovement.getYieldIncrease(yy) > 0)
			{
				isLivestockImprovment = true;
			}
		}

		if (isLivestockImprovment)
		{
			for (int iI = 0; iI < NUM_CITY_PLOTS; iI++)
			{
				CvPlot* pLoopPlot = pWorkingCity->getCityIndexPlot(iI);
				if (pLoopPlot != NULL)
				{
					if (pLoopPlot->getImprovementType() == eImprovement)
					{
						return false;
					}
				}
			}
		}
	}
	// R&R, ray, Improving AI logic for Improvements, END


	if ((getBonusType() != NO_BONUS) && GC.getImprovementInfo(eImprovement).isImprovementBonusMakesValid(getBonusType()))
	{
		return true;
	}

	if (GC.getImprovementInfo(eImprovement).isRequiresFlatlands() && !isFlatlands())
	{
		return false;
	}

	if (GC.getImprovementInfo(eImprovement).isRequiresFeature() && (getFeatureType() == NO_FEATURE))
	{
		return false;
	}

	if (GC.getImprovementInfo(eImprovement).isHillsMakesValid() && (isHills() || isPeak()))
	{
		bValid = true;
	}

	if (GC.getImprovementInfo(eImprovement).isRiverSideMakesValid() && isRiverSide())
	{
		bValid = true;
	}

	if (GC.getImprovementInfo(eImprovement).getTerrainMakesValid(getTerrainType()))
	{
		bValid = true;
	}

	if ((getFeatureType() != NO_FEATURE) && GC.getImprovementInfo(eImprovement).getFeatureMakesValid(getFeatureType()))
	{
		bValid = true;
	}

	if (!bValid)
	{
		return false;
	}

	if (GC.getImprovementInfo(eImprovement).isRequiresRiverSide())
	{
		bValid = false;

		for (iI = 0; iI < NUM_CARDINALDIRECTION_TYPES; ++iI)
		{
			pLoopPlot = plotCardinalDirection(getX_INLINE(), getY_INLINE(), ((CardinalDirectionTypes)iI));

			if (pLoopPlot != NULL)
			{
				if (isRiverCrossing(directionXY(this, pLoopPlot)))
				{
					if (pLoopPlot->getImprovementType() != eImprovement)
					{
						bValid = true;
						break;
					}
				}
			}
		}

		if (!bValid)
		{
			return false;
		}
	}

	bool bPrereq = false;
	bValid = false;
	for (iI = 0; iI < NUM_YIELD_TYPES && !bValid; ++iI)
	{
		int iRequired = GC.getImprovementInfo(eImprovement).getPrereqNatureYield(iI);
		if (iRequired > 0)
		{
			bPrereq = true;
			if (calculateNatureYield(((YieldTypes)iI), eTeam, /* build feature removal detection - Nightinggale */ bIgnoreFeature) >= iRequired)
			{
				bValid = true;
			}
		}
	}

	if (bPrereq && !bValid)
	{
		return false;
	}

	return true;
}


bool CvPlot::canBuild(BuildTypes eBuild, PlayerTypes ePlayer, bool bTestVisible) const
{
	ImprovementTypes eImprovement;
	ImprovementTypes eFinalImprovementType;
	RouteTypes eRoute;
	bool bValid;

	if(GC.getUSE_CAN_BUILD_CALLBACK())
	{
		CyArgsList argsList;
		argsList.add(getX_INLINE());
		argsList.add(getY_INLINE());
		argsList.add((int)eBuild);
		argsList.add((int)ePlayer);
		long lResult=0;
		gDLL->getPythonIFace()->callFunction(PYGameModule, "canBuild", argsList.makeFunctionArgs(), &lResult);
		if (lResult >= 1)
		{
			return true;
		}
		else if (lResult == 0)
		{
			return false;
		}
	}

	if (eBuild == NO_BUILD)
	{
		return false;
	}

	bValid = false;

	eImprovement = ((ImprovementTypes)(GC.getBuildInfo(eBuild).getImprovement()));

	if (eImprovement != NO_IMPROVEMENT)
	{
		// build feature removal detection - start - Nightinggale
		bool bRemoveFeature = false;
		if (getFeatureType() != NO_FEATURE)
		{
			bRemoveFeature = GC.getBuildInfo(eBuild).isFeatureRemove(getFeatureType());
		}

		// build feature removal detection - end - Nightinggale

		if (!canHaveImprovement(eImprovement, GET_PLAYER(ePlayer).getTeam(), bTestVisible, /* build feature removal detection - Nightinggale */ bRemoveFeature))
		{
			return false;
		}

		// Super Forts begin *build*
		if (GC.getImprovementInfo(eImprovement).getUniqueRange() > 0)
		{
			int iUniqueRange = GC.getImprovementInfo(eImprovement).getUniqueRange();
			for (int iDX = -iUniqueRange; iDX <= iUniqueRange; iDX++) 
			{
				for (int iDY = -iUniqueRange; iDY <= iUniqueRange; iDY++)
				{
					CvPlot *pLoopPlot = plotXY(getX_INLINE(), getY_INLINE(), iDX, iDY);
					if (pLoopPlot != NULL && pLoopPlot->getImprovementType() != NO_IMPROVEMENT)
					{
						if (finalImprovementUpgrade(pLoopPlot->getImprovementType()) == finalImprovementUpgrade(eImprovement))
						{
							return false;
						}
					}
				}
			}
		}
		// Super Forts end

		if (getImprovementType() != NO_IMPROVEMENT)
		{
			if (GC.getImprovementInfo(getImprovementType()).isPermanent())
			{
				return false;
			}

			// Super Forts begin *AI_worker* - prevent forts from being built over when outside culture range
			if (GC.getImprovementInfo(getImprovementType()).isActsAsCity())
			{
				if (!isWithinCultureRange(ePlayer) && !(getCultureRangeForts(ePlayer) > 1))
				{
					return false;
				}
			}
			// Super Forts end

			if (getImprovementType() == eImprovement)
			{
				return false;
			}

			eFinalImprovementType = finalImprovementUpgrade(getImprovementType());

			if (eFinalImprovementType != NO_IMPROVEMENT)
			{
				if (eFinalImprovementType == finalImprovementUpgrade(eImprovement))
				{
					return false;
				}
			}
		}

		if (!bTestVisible)
		{
			if (GET_PLAYER(ePlayer).getTeam() != getTeam())
			{
				//outside borders can't be built in other's culture
				if (GC.getImprovementInfo(eImprovement).isOutsideBorders())
				{
					if (getTeam() != NO_TEAM)
					{
						return false;
					}
				}
				else //only buildable in own culture
				{
					return false;
				}
			}
			//R&R, ray, preventing exploit to build Monasteries or Forts directly next to a Native Village in own culture - START
			//still allowing to build Monasteries and Forts next to own cities
			else
			{
				if (GC.getImprovementInfo(eImprovement).isOutsideBorders())
				{
					bool bFoundNeighbourCity = false;
					for (int i = 0; i < NUM_DIRECTION_TYPES; ++i)
					{
						CvPlot* pLoopPlot = ::plotDirection(getX_INLINE(), getY_INLINE(), (DirectionTypes) i);
						if (pLoopPlot != NULL && pLoopPlot->getTeam() != GET_PLAYER(ePlayer).getTeam())
						{
							if (pLoopPlot->isCity())
							{
								bFoundNeighbourCity = true;
								break;
							}
						}
					}
					if (bFoundNeighbourCity)
					{
						return false;
					}
				}
			}
			//R&R, ray, preventing exploit to build directly next to a Native Village in own culture - END


			//R&R, vetiarvind, Super Forts begin *AI_worker* - prevent workers from two different players from building a fort in the same plot 
			if(GC.getImprovementInfo(eImprovement).isActsAsCity())
			{
				CLLNode<IDInfo>* pUnitNode = headUnitNode();
				CvUnit* pLoopUnit;
				
				while (pUnitNode != NULL)
				{
					pLoopUnit = ::getUnit(pUnitNode->m_data);
					pUnitNode = nextUnitNode(pUnitNode);
					if(pLoopUnit != NULL && pLoopUnit->getOwner() != ePlayer)	
					{
						if(pLoopUnit->getBuildType() != NO_BUILD)
						{
							ImprovementTypes eImprovementBuild = (ImprovementTypes)(GC.getBuildInfo(pLoopUnit->getBuildType()).getImprovement());
							if(eImprovementBuild != NO_IMPROVEMENT)
							{
								if(GC.getImprovementInfo(eImprovementBuild).isActsAsCity())
								{
									return false;
								}
							}
						}
					}
				}
			}
			//R&R, vetiarvind, Super Forts end *AI_worker*
		}

		bValid = true;
	}

	eRoute = ((RouteTypes)(GC.getBuildInfo(eBuild).getRoute()));

	if (eRoute != NO_ROUTE)
	{
		if (isPeak())
		{
			bool bFoundNonPeak = false;
			for (int i = 0; i < NUM_DIRECTION_TYPES; ++i)
			{
				CvPlot* pLoopPlot = ::plotDirection(getX_INLINE(), getY_INLINE(), (DirectionTypes) i);
				if (pLoopPlot != NULL)
				{
					if (!pLoopPlot->isPeak())
					{
						bFoundNonPeak = true;
						break;
					}
				}
			}
			if (!bFoundNonPeak)
			{
				return false;
			}
		}

		if (getRouteType() != NO_ROUTE)
		{
			if (GC.getRouteInfo(getRouteType()).getValue() >= GC.getRouteInfo(eRoute).getValue())
			{
				return false;
			}
		}

		// R&R, ray, one Route Type after the other - START
		if (getRouteType() == NO_ROUTE && GC.getRouteInfo(eRoute).getValue() > 1)
		{
			return false;
		}
		// R&R, ray, one Route Type after the other - START

		bValid = true;
	}

	if (getFeatureType() != NO_FEATURE)
	{
		if (GC.getBuildInfo(eBuild).isFeatureRemove(getFeatureType()))
		{
			if (isOwned() && (GET_PLAYER(ePlayer).getTeam() != getTeam()) && !atWar(GET_PLAYER(ePlayer).getTeam(), getTeam()))
			{
				return false;
			}

			bValid = true;
		}
	}

	// R&R, ray, Terraforming Features - START
	TerrainTypes ePrereqTerrain = (TerrainTypes)(GC.getBuildInfo(eBuild).getPrereqTerrain());
	if (ePrereqTerrain != NO_TERRAIN)
	{
		// check if Terrain fits
		if (getTerrainType() != ePrereqTerrain)
		{
			return false;
		}
		// only inside own borders
		if (GET_PLAYER(ePlayer).getTeam() != getTeam())
		{
			return false;
		}

		bValid = true;
	}

	FeatureTypes eResultFeature = (FeatureTypes)(GC.getBuildInfo(eBuild).getResultFeature());
	if (eResultFeature != NO_FEATURE)
	{
		// check if there is not already Feature
		if (getFeatureType() != NO_FEATURE)
		{
			return false;
		}
		// only inside own borders
		if (GET_PLAYER(ePlayer).getTeam() != getTeam())
		{
			return false;
		}

		bValid = true;
	}
	// R&R, ray, Terraforming Features - END
	return bValid;
}


int CvPlot::getBuildTime(BuildTypes eBuild) const
{
	int iTime;

	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	iTime = GC.getBuildInfo(eBuild).getTime();

	if (getFeatureType() != NO_FEATURE)
	{
		iTime += GC.getBuildInfo(eBuild).getFeatureTime(getFeatureType());
	}

	iTime *= std::max(0, (GC.getTerrainInfo(getTerrainType()).getBuildModifier() + 100));
	iTime /= 100;

	iTime *= GC.getGameSpeedInfo(GC.getGameINLINE().getGameSpeedType()).getGrowthPercent();
	iTime /= 100;

	iTime *= GC.getEraInfo(GC.getGameINLINE().getStartEra()).getGrowthPercent();
	iTime /= 100;

	return iTime;
}


int CvPlot::getBuildTurnsLeft(BuildTypes eBuild, int iNowExtra, int iThenExtra) const
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;
	int iNowBuildRate;
	int iThenBuildRate;
	int iBuildLeft;
	int iTurnsLeft;

	iNowBuildRate = iNowExtra;
	iThenBuildRate = iThenExtra;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if (pLoopUnit->getBuildType() == eBuild)
		{
			if (pLoopUnit->canMove())
			{
				iNowBuildRate += pLoopUnit->workRate(false);
			}
			iThenBuildRate += pLoopUnit->workRate(true);
		}
	}

	if (iThenBuildRate == 0)
	{
		//this means it will take forever under current circumstances
		return MAX_INT;
	}

	iBuildLeft = getBuildTime(eBuild);

	iBuildLeft -= getBuildProgress(eBuild);
	iBuildLeft -= iNowBuildRate;

	iBuildLeft = std::max(0, iBuildLeft);

	iTurnsLeft = (iBuildLeft / iThenBuildRate);

	if ((iTurnsLeft * iThenBuildRate) < iBuildLeft)
	{
		iTurnsLeft++;
	}

	iTurnsLeft++;

	return std::max(1, iTurnsLeft);
}


int CvPlot::getFeatureYield(BuildTypes eBuild, YieldTypes eYield, TeamTypes eTeam, CvCity** ppCity) const
{
	if (getFeatureType() == NO_FEATURE)
	{
		return 0;
	}

	if (*ppCity == NULL)
	{
		*ppCity = getWorkingCity();

		if (*ppCity == NULL)
		{
			*ppCity = GC.getMapINLINE().findCity(getX_INLINE(), getY_INLINE(), NO_PLAYER, eTeam, false);
		}
	}

	if (*ppCity == NULL)
	{
		return 0;
	}

	int iProduction = GC.getBuildInfo(eBuild).getFeatureYield(getFeatureType(), eYield);
	const int iSafeDistance = 2;
	int iStep = iProduction / std::max(iSafeDistance, GC.getDefineINT("FEATURE_PRODUCTION_YIELD_MAX_DISTANCE"));
	iProduction -= std::max(0, plotDistance(getX_INLINE(), getY_INLINE(), (*ppCity)->getX_INLINE(), (*ppCity)->getY_INLINE()) - iSafeDistance) * iStep;
	
	iProduction *= GC.getGameSpeedInfo(GC.getGameINLINE().getGameSpeedType()).getGrowthPercent();
	iProduction /= 100;

	if (getTeam() != eTeam)
	{
		iProduction *= GC.getDefineINT("DIFFERENT_TEAM_FEATURE_PRODUCTION_PERCENT");
		iProduction /= 100;
	}

	return std::max(0, iProduction);
}


CvUnit* CvPlot::getBestDefender(PlayerTypes eOwner, PlayerTypes eAttackingPlayer, const CvUnit* pAttacker, bool bTestAtWar, bool bTestPotentialEnemy, bool bTestCanMove) const
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;
	CvUnit* pBestUnit;

	pBestUnit = NULL;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if (pLoopUnit->isOnMap() && !pLoopUnit->isCargo())
		{
			if ((eOwner == NO_PLAYER) || (pLoopUnit->getOwnerINLINE() == eOwner))
			{
				if ((eAttackingPlayer == NO_PLAYER) || !(pLoopUnit->isInvisible(GET_PLAYER(eAttackingPlayer).getTeam(), false)))
				{
					if (!bTestAtWar || eAttackingPlayer == NO_PLAYER || pLoopUnit->isEnemy(GET_PLAYER(eAttackingPlayer).getTeam(), this) || (NULL != pAttacker && pAttacker->isEnemy(GET_PLAYER(pLoopUnit->getOwnerINLINE()).getTeam(), this)))
					{
						if (!bTestPotentialEnemy || (eAttackingPlayer == NO_PLAYER) ||  pLoopUnit->isPotentialEnemy(GET_PLAYER(eAttackingPlayer).getTeam(), this) || (NULL != pAttacker && pAttacker->isPotentialEnemy(GET_PLAYER(pLoopUnit->getOwnerINLINE()).getTeam(), this)))
						{
							if (!bTestCanMove || pLoopUnit->canMove() && !(pLoopUnit->isCargo()))
							{
								if (pLoopUnit->isBetterDefenderThan(pBestUnit, pAttacker, true))
								{
									pBestUnit = pLoopUnit;
								}
							}
						}
					}
				}
			}
		}
	}

	return pBestUnit;
}

///<summary>Efficient alternative to getBestDefender() in case we want to check for any defender being present</summary>
bool CvPlot::hasDefender(bool bCheckCanAttack, PlayerTypes eOwner, PlayerTypes eAttackingPlayer, const CvUnit* pAttacker, bool bTestAtWar, bool bTestPotentialEnemy, bool bTestCanMove) const
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;
	CvUnit* pBestUnit;

	pBestUnit = NULL;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if (pLoopUnit->isOnMap() && !pLoopUnit->isCargo())
		{
			if ((eOwner == NO_PLAYER) || (pLoopUnit->getOwnerINLINE() == eOwner))
			{
				if ((eAttackingPlayer == NO_PLAYER) || !(pLoopUnit->isInvisible(GET_PLAYER(eAttackingPlayer).getTeam(), false)))
				{
					if (!bTestAtWar || eAttackingPlayer == NO_PLAYER || pLoopUnit->isEnemy(GET_PLAYER(eAttackingPlayer).getTeam(), this) || (NULL != pAttacker && pAttacker->isEnemy(GET_PLAYER(pLoopUnit->getOwnerINLINE()).getTeam(), this)))
					{
						if (!bTestPotentialEnemy || (eAttackingPlayer == NO_PLAYER) || pLoopUnit->isPotentialEnemy(GET_PLAYER(eAttackingPlayer).getTeam(), this) || (NULL != pAttacker && pAttacker->isPotentialEnemy(GET_PLAYER(pLoopUnit->getOwnerINLINE()).getTeam(), this)))
						{
							if (!bTestCanMove || pLoopUnit->canMove() && !(pLoopUnit->isCargo()))
							{
								if (!bCheckCanAttack || (pAttacker == NULL) || (pAttacker->canAttack()))
								{ 
									// found a valid defender
									return true;
								}
							}
						}
					}
				}
			}
		}
	}

	// there are no defenders
	return false;
}

// returns a sum of the strength (adjusted by firepower) of all the units on a plot
int CvPlot::AI_sumStrength(PlayerTypes eOwner, PlayerTypes eAttackingPlayer, DomainTypes eDomainType, bool bDefensiveBonuses, bool bTestAtWar, bool bTestPotentialEnemy) const
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;
	int	strSum = 0;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if ((eOwner == NO_PLAYER) || (pLoopUnit->getOwnerINLINE() == eOwner))
		{
			if ((eAttackingPlayer == NO_PLAYER) || !(pLoopUnit->isInvisible(GET_PLAYER(eAttackingPlayer).getTeam(), false)))
			{
				if (!bTestAtWar || (eAttackingPlayer == NO_PLAYER) || atWar(GET_PLAYER(eAttackingPlayer).getTeam(), pLoopUnit->getTeam()))
				{
					if (!bTestPotentialEnemy || (eAttackingPlayer == NO_PLAYER) || pLoopUnit->isPotentialEnemy(GET_PLAYER(eAttackingPlayer).getTeam(), this))
					{
						// we may want to be more sophisticated about domains
						// somewhere we need to check to see if this is a city, if so, only land units can defend here, etc
						if (eDomainType == NO_DOMAIN || (pLoopUnit->getDomainType() == eDomainType))
						{
							const CvPlot* pPlot = NULL;
							if (bDefensiveBonuses)
								pPlot = this;
							strSum += pLoopUnit->currEffectiveStr(pPlot, NULL);
						}
					}
				}
			}
		}
	}

	return strSum;
}


CvUnit* CvPlot::getSelectedUnit() const
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if (pLoopUnit->IsSelected())
		{
			return pLoopUnit;
		}
	}

	return NULL;
}


int CvPlot::getUnitPower(PlayerTypes eOwner) const
{
	int iCount = 0;
	CLLNode<IDInfo>* pUnitNode = headUnitNode();
	while (pUnitNode != NULL)
	{
		CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if ((eOwner == NO_PLAYER) || (pLoopUnit->getOwnerINLINE() == eOwner))
		{
			iCount += pLoopUnit->getPower();
		}
	}

	CvCity* pCity = getPlotCity();
	if (pCity != NULL && pCity->getOwnerINLINE() == eOwner)
	{
		for (int i = 0; i < pCity->getPopulation(); ++i)
		{
			iCount += pCity->getPopulationUnitByIndex(i)->getPower();
		}
	}

	return iCount;
}

// Super Forts begin *bombard*
bool CvPlot::isBombardable(const CvUnit* pUnit) const
{
	if (NULL != pUnit && !pUnit->isEnemy(getTeam()))
	{
		return false;
	}

	ImprovementTypes eImprovement = getImprovementType();
	if(eImprovement != NO_IMPROVEMENT)	
	{
		if(GC.getImprovementInfo(eImprovement).isBombardable())
		{
			return (getDefenseDamage() < GC.getImprovementInfo(eImprovement).getDefenseModifier());
		}
	}
	return false;
}

bool CvPlot::isBombarded() const
{
	return m_bBombarded;
}

void CvPlot::setBombarded(bool bNewValue)
{
	m_bBombarded = bNewValue;
}

int CvPlot::getDefenseDamage() const																
{
	return m_iDefenseDamage;
}

void CvPlot::changeDefenseDamage(int iChange)
{
	if ((iChange != 0) && (getImprovementType() != NO_IMPROVEMENT))
	{
		m_iDefenseDamage = range((m_iDefenseDamage + iChange), 0, GC.getImprovementInfo(getImprovementType()).getDefenseModifier());

		if (iChange > 0)
		{
			setBombarded(true);
		}
	}
}
// Super Forts end

// Super Forts begin *culture*
int CvPlot::getCultureRangeForts(PlayerTypes ePlayer) const
{
	if (NULL == m_aiCultureRangeForts)
	{
		return 0;
	}

	return m_aiCultureRangeForts[ePlayer];
}

void CvPlot::setCultureRangeForts(PlayerTypes ePlayer, int iNewValue)
{
	if (getCultureRangeForts(ePlayer) != iNewValue)
	{
		if(NULL == m_aiCultureRangeForts)
		{
			m_aiCultureRangeForts = new short[MAX_PLAYERS];
			for (int iI = 0; iI < MAX_PLAYERS; ++iI)
			{
				m_aiCultureRangeForts[iI] = 0;
			}
		}

		m_aiCultureRangeForts[ePlayer] = iNewValue;
		
		if(getCulture(ePlayer) == 0)
		{
			changeCulture(ePlayer, 1, false);
		}
	}
}

void CvPlot::changeCultureRangeForts(PlayerTypes ePlayer, int iChange)
{
	if (0 != iChange)
	{
		setCultureRangeForts(ePlayer, (getCultureRangeForts(ePlayer) + iChange));
	}
}

bool CvPlot::isWithinFortCultureRange(PlayerTypes ePlayer) const
{
	return (getCultureRangeForts(ePlayer) > 0);
}

void CvPlot::changeCultureRangeFortsWithinRange(PlayerTypes ePlayer, int iChange, int iRange, bool bUpdate)
{
	CvPlot* pLoopPlot;
	int iDX, iDY;
	int iCultureDistance;

	if ((0 != iChange) && (iRange >= 0))
	{
		for (iDX = -iRange; iDX <= iRange; iDX++)
		{
			for (iDY = -iRange; iDY <= iRange; iDY++)
			{
				iCultureDistance = plotDistance(0, 0, iDX, iDY);

				if(iCultureDistance <= iRange)
				{
					pLoopPlot = plotXY(getX_INLINE(), getY_INLINE(), iDX, iDY);

					if (pLoopPlot != NULL)
					{
						pLoopPlot->changeCultureRangeForts(ePlayer, iChange);

						if(bUpdate)
						{
							pLoopPlot->updateCulture(true);
						}
					}
				}
			}
		}
	}
}

void CvPlot::doImprovementCulture()
{
	CvPlot* pLoopPlot;
	int iDX, iDY;
	int iCultureDistance, iCulture, iCultureRange;
	ImprovementTypes eImprovement;
	PlayerTypes ePlayer;

	eImprovement = getImprovementType();
	if (eImprovement != NO_IMPROVEMENT)
	{
		ePlayer = getOwnerINLINE();
		if(ePlayer != NO_PLAYER)
		{
			iCulture = GC.getImprovementInfo(eImprovement).getCulture();
			if(iCulture > 0)
			{
				iCultureRange = GC.getImprovementInfo(eImprovement).getCultureRange();
				
				if(iCultureRange > 0)
				{
					for (iDX = -iCultureRange; iDX <= iCultureRange; iDX++)
					{
						for (iDY = -iCultureRange; iDY <= iCultureRange; iDY++)
						{
							iCultureDistance = plotDistance(0, 0, iDX, iDY);

							if(iCultureDistance <= iCultureRange)
							{
								pLoopPlot = plotXY(getX_INLINE(), getY_INLINE(), iDX, iDY);

								if (pLoopPlot != NULL)
								{
									int iChange = ((iCultureRange - ((iCultureDistance == 0) ? 1 : iCultureDistance))*iCulture) + iCulture;
									pLoopPlot->changeCulture(ePlayer,iChange,false);
								}
							}
						}
					}
				}
				else
				{
					changeCulture(ePlayer,iCulture,false);
				}
			}
		}
	}
}
// Super Forts end

// Super Forts begin *canal* *choke*
//R&R mod, vetiarvind super forts merge, don't count region plots
/*
int CvPlot::countRegionPlots(const CvPlot* pInvalidPlot) const
{
	int iCount = 0;
	int iInvalidPlot = (pInvalidPlot == NULL) ? 0 : GC.getMapINLINE().plotNum(pInvalidPlot->getX_INLINE(), pInvalidPlot->getY_INLINE()) + 1;
	FAStar* pRegionFinder = gDLL->getFAStarIFace()->create();
	gDLL->getFAStarIFace()->Initialize(pRegionFinder, GC.getMapINLINE().getGridWidthINLINE(), GC.getMapINLINE().getGridHeightINLINE(), GC.getMapINLINE().isWrapXINLINE(), GC.getMapINLINE().isWrapYINLINE(), 
		NULL, NULL, NULL, stepValid, NULL, countPlotGroup, NULL);
	gDLL->getFAStarIFace()->SetData(pRegionFinder, &iCount);
	// Note to self: for GeneratePath() should bReuse be true or false?
	gDLL->getFAStarIFace()->GeneratePath(pRegionFinder, getX_INLINE(), getY_INLINE(), -1, -1, false, iInvalidPlot, false);
	gDLL->getFAStarIFace()->destroy(pRegionFinder);
	return iCount;
}*/

int CvPlot::countAdjacentPassableSections(bool bWater) const
{
	CvPlot* pAdjacentPlot;
	int iPassableSections = 0;
	bool bInPassableSection = false;

	// Are we looking for water passages or land passages?
	if(bWater)
	{
		bool bPlotIsWater = isWater();
		// This loop is for water
		for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));
			if(pAdjacentPlot != NULL)
			{
				if(pAdjacentPlot->isWater())
				{
					// Don't count diagonal hops across land isthmus
					if (bPlotIsWater && !isCardinalDirection((DirectionTypes)iI))
					{
						if (!(GC.getMapINLINE().plotINLINE(getX_INLINE(), pAdjacentPlot->getY_INLINE())->isWater()) && !(GC.getMapINLINE().plotINLINE(pAdjacentPlot->getX_INLINE(), getY_INLINE())->isWater()))
						{
							continue;
						}
					}
					if(pAdjacentPlot->isImpassable())
					{
						if(isCardinalDirection((DirectionTypes)iI))
						{
							bInPassableSection = false;
						}
					}
					else if(!bInPassableSection)
					{
						bInPassableSection = true;
						++iPassableSections;
					}
				}
				else
				{
					bInPassableSection = false;
				}
			}
		}
	}
	else
	{
		// This loop is for land
		for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));
			if(pAdjacentPlot != NULL)
			{
				if(pAdjacentPlot->isWater() || pAdjacentPlot->isImpassable())
				{	
					if(isCardinalDirection((DirectionTypes)iI))
					{
						bInPassableSection = false;
					}
				}
				else if(!bInPassableSection)
				{
					bInPassableSection = true;
					++iPassableSections;
				}
			}
		}
	}
	// Corner Case Correction
	pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_NORTH);
	if(pAdjacentPlot != NULL && (bWater == pAdjacentPlot->isWater()) && !pAdjacentPlot->isImpassable())
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_NORTHWEST);
		if(pAdjacentPlot != NULL && (bWater == pAdjacentPlot->isWater()))
		{
			if(pAdjacentPlot->isImpassable())
			{
				pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_WEST);
				if(pAdjacentPlot != NULL && (bWater == pAdjacentPlot->isWater()) && !pAdjacentPlot->isImpassable())
				{
					--iPassableSections;
				}
			}
			else
			{
				--iPassableSections;
			}
		}
		else if(!bWater)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_WEST);
			if(pAdjacentPlot != NULL && !pAdjacentPlot->isWater() && !pAdjacentPlot->isImpassable())
			{
				--iPassableSections;
			}
		}
	}
	return iPassableSections;
}

int CvPlot::countImpassableCardinalDirections() const
{
	CvPlot* pAdjacentPlot;
	int iCount = 0;
	for(int iI = 0; iI < NUM_CARDINALDIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotCardinalDirection(getX_INLINE(), getY_INLINE(), ((CardinalDirectionTypes)iI));
		if(pAdjacentPlot != NULL)
		{
			if(pAdjacentPlot->isImpassable() || (area() != pAdjacentPlot->area()))
			{
				++iCount;
			}
		}
	}
	return iCount;
}
// Super Forts end

// Super Forts begin *canal*
int CvPlot::getCanalValue() const																
{
	return m_iCanalValue;
}

void CvPlot::setCanalValue(int iNewValue)
{
	m_iCanalValue = iNewValue;
}

void CvPlot::calculateCanalValue()
{
	bool bInWaterSection;
	CvPlot *pAdjacentPlot, *apPlotsToCheck[4];
	int iWaterSections, iPlotsFound, iMaxDistance;
	int iCanalValue = 0;

	if(isCoastalLand() && !isImpassable())
	{
		iWaterSections = countAdjacentPassableSections(true);
		if(iWaterSections > 1)
		{
			iMaxDistance = 0;
			iPlotsFound = 0;
			bInWaterSection = false;
			// Find appropriate plots to be used for path distance calculations
			for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
			{
				pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));
				if(pAdjacentPlot != NULL)
				{
					if(pAdjacentPlot->isWater())
					{
						if(pAdjacentPlot->isImpassable())
						{
							if(isCardinalDirection((DirectionTypes)iI))
							{
								bInWaterSection = false;
							}
						}
						else if(!bInWaterSection)
						{
							bInWaterSection = true;
							apPlotsToCheck[iPlotsFound] = pAdjacentPlot;
							if((++iPlotsFound) == iWaterSections)
								break;
						}
					}
					else
					{
						bInWaterSection = false;
					}
				}
			}
			// Find the max path distance out of all possible pairs of plots
			for (int iI = 0; iI < (iPlotsFound - 1); ++iI)
			{
				for (int iJ = iI + 1; iJ < iPlotsFound; ++iJ)
				{
					if(!apPlotsToCheck[iI]->isLake() || !apPlotsToCheck[iJ]->isLake())
					{
						int iDistance = GC.getMapINLINE().calculatePathDistance(apPlotsToCheck[iI], apPlotsToCheck[iJ]);
						if(iDistance == -1)
						{
						
							// If no path was found then value is based off the number of plots in the region minus a minimum area
							//R&R mod, vetiarvind merge, don't count region plots
							/*iDistance = std::min(apPlotsToCheck[iI]->countRegionPlots(), apPlotsToCheck[iJ]->countRegionPlots()) - 7;
							iDistance *= 4;
							*/
						}
						else
						{
							// Path already would have required 2 steps, and I don't care that much about saving just 1 or 2 moves
							iDistance -= 4;
						}
						if(iDistance > iMaxDistance)
						{
							iMaxDistance = iDistance;
						}
					}
				}
			}
			iCanalValue = iMaxDistance * (iPlotsFound - 1);
		}
	}

	setCanalValue(iCanalValue);
}
// Super Forts end

// Super Forts begin *choke*
int CvPlot::getChokeValue() const
{
	return m_iChokeValue;
}

void CvPlot::setChokeValue(int iNewValue)
{
	m_iChokeValue = iNewValue;
}

void CvPlot::calculateChokeValue()
{
	bool bInPassableSection;
	CvPlot *pAdjacentPlot, *apPlotsToCheck[4];
	int iPassableSections, iPlotsFound, iMaxDistance;
	int iChokeValue = 0;
	bool bWater = isWater();

	if(!isImpassable() && countImpassableCardinalDirections() > 1)
	{
		iPassableSections = countAdjacentPassableSections(bWater);
		if(iPassableSections > 1)
		{
			iMaxDistance = 0;
			iPlotsFound = 0;
			bInPassableSection = false;
			// Find appropriate plots to be used for path distance calculations
			for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
			{
				pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));
				if(pAdjacentPlot != NULL)
				{
					if(pAdjacentPlot->isWater() == bWater)
					{	
						// Don't count diagonal hops across land isthmus
						if (bWater && !isCardinalDirection((DirectionTypes)iI))
						{
							if (!(GC.getMapINLINE().plotINLINE(getX_INLINE(), pAdjacentPlot->getY_INLINE())->isWater()) && !(GC.getMapINLINE().plotINLINE(pAdjacentPlot->getX_INLINE(), getY_INLINE())->isWater()))
							{
								continue;
							}
						}
						if(pAdjacentPlot->isImpassable())
						{
							if(isCardinalDirection((DirectionTypes)iI))
							{
								bInPassableSection = false;
							}
						}
						else if(!bInPassableSection)
						{
							bInPassableSection = true;
							apPlotsToCheck[iPlotsFound] = pAdjacentPlot;
							if((++iPlotsFound) == iPassableSections)
								break;
						}
					}
					else if(bWater || isCardinalDirection((DirectionTypes)iI))
					{
						bInPassableSection = false;
					}
				}
			}
			// Find the max path distance out of all possible pairs of plots
			for (int iI = 0; iI < (iPlotsFound - 1); ++iI)
			{
				for (int iJ = iI + 1; iJ < iPlotsFound; ++iJ)
				{
					int iDistance = GC.getMapINLINE().calculatePathDistance(apPlotsToCheck[iI], apPlotsToCheck[iJ], this);
					if(iDistance == -1)
					{
						// If no path was found then value is based off the number of plots in the region minus a minimum area
						//R&R mod, vetiarvind merge, don't count region plots
						/*iDistance = std::min(apPlotsToCheck[iI]->countRegionPlots(this), apPlotsToCheck[iJ]->countRegionPlots(this)) - 4;
						iDistance *= 4;*/
					}
					else
					{
						// Path already would have required 2 steps, but we forced the enemy to go another way so there is some value
						iDistance -= 1;
					}
					if(iDistance > iMaxDistance)
					{
						iMaxDistance = iDistance;
					}
				}
			}
			iChokeValue = iMaxDistance * (iPlotsFound - 1);
		}
	}

	setChokeValue(iChokeValue);
}
// Super Forts end

int CvPlot::defenseModifier(TeamTypes eDefender, bool bHelp) const
{
	CvCity* pCity;
	ImprovementTypes eImprovement;
	int iModifier;

	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	iModifier = ((getFeatureType() == NO_FEATURE) ? GC.getTerrainInfo(getTerrainType()).getDefenseModifier() : GC.getFeatureInfo(getFeatureType()).getDefenseModifier());

	if (isHills() || isPeak())
	{
		iModifier += GC.getHILLS_EXTRA_DEFENSE();
	}

	if (bHelp)
	{
		eImprovement = getRevealedImprovementType(GC.getGameINLINE().getActiveTeam(), false);
	}
	else
	{
		eImprovement = getImprovementType();
	}

	if (eImprovement != NO_IMPROVEMENT)
	{
		if (eDefender != NO_TEAM && (getTeam() == NO_TEAM || GET_TEAM(eDefender).isFriendlyTerritory(getTeam())))
		{
			// Super Forts begin *bombard*
			iModifier += GC.getImprovementInfo(eImprovement).getDefenseModifier() - getDefenseDamage();
			// iModifier += GC.getImprovementInfo(eImprovement).getDefenseModifier(); - Original code
			// Super Forts end			
		}
	}

	if (!bHelp)
	{
		pCity = getPlotCity();

		if (pCity != NULL)
		{
			iModifier += pCity->getDefenseModifier();
		}
	}

	return iModifier;
}


int CvPlot::movementCost(const CvUnit* pUnit, const CvPlot* pFromPlot) const
{
	int iRegularCost;
	int iRouteCost;
	int iRouteFlatCost;

	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	if (pUnit->flatMovementCost())
	{
		return GC.getMOVE_DENOMINATOR();
	}

	if (!isRevealed(pUnit->getTeam(), false))
	{
		if (pUnit->getDomainType() == DOMAIN_SEA)
		{
			return GC.getMOVE_DENOMINATOR();
		}
		else
		{
			return pUnit->maxMoves();
		}
	}

	if (!pFromPlot->isValidDomainForLocation(*pUnit))
	{
		return pUnit->maxMoves();
	}

	if (!isValidDomainForAction(*pUnit))
	{
		return GC.getMOVE_DENOMINATOR();
	}

	FAssert(pUnit->getDomainType() != DOMAIN_IMMOBILE);

	if (pUnit->ignoreTerrainCost())
	{
		iRegularCost = 1;
	}
	else
	{
		iRegularCost = ((getFeatureType() == NO_FEATURE) ? GC.getTerrainInfo(getTerrainType()).getMovementCost() : GC.getFeatureInfo(getFeatureType()).getMovementCost());

		if (isHills())
		{
			iRegularCost += GC.getHILLS_EXTRA_MOVEMENT();
		}

		if (isPeak())
		{
			iRegularCost += GC.getPEAK_EXTRA_MOVEMENT();
		}

		if (iRegularCost > 0)
		{
			iRegularCost = std::max(1, (iRegularCost - pUnit->getExtraMoveDiscount()));
		}
	}

	bool bHasTerrainCost = (iRegularCost > 0);

	iRegularCost = std::min(iRegularCost, pUnit->baseMoves()) * GC.getMOVE_DENOMINATOR();

	if (bHasTerrainCost)
	{
		if (((getFeatureType() == NO_FEATURE) ? pUnit->isTerrainDoubleMove(getTerrainType()) : pUnit->isFeatureDoubleMove(getFeatureType())) ||
			((isHills() || isPeak()) && pUnit->isHillsDoubleMove()))
		{
			iRegularCost /= 2;
		}
	}

	if (pFromPlot->isValidRoute(pUnit) && isValidRoute(pUnit))
	{
		iRouteCost = std::max((GC.getRouteInfo(pFromPlot->getRouteType()).getMovementCost()),
			               (GC.getRouteInfo(getRouteType()).getMovementCost()));
		iRouteFlatCost = std::max((GC.getRouteInfo(pFromPlot->getRouteType()).getFlatMovementCost() * pUnit->baseMoves()),
			                   (GC.getRouteInfo(getRouteType()).getFlatMovementCost() * pUnit->baseMoves()));
	}
	else
	{
		iRouteCost = MAX_INT;
		iRouteFlatCost = MAX_INT;
	}

	return std::max(1, std::min(iRegularCost, std::min(iRouteCost, iRouteFlatCost)));
}

bool CvPlot::isAdjacentOwned() const
{
	CvPlot* pAdjacentPlot;
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->isOwned())
			{
				return true;
			}
		}
	}

	return false;
}


bool CvPlot::isAdjacentPlayer(PlayerTypes ePlayer, bool bLandOnly) const
{
	CvPlot* pAdjacentPlot;
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->getOwnerINLINE() == ePlayer)
			{
				if (!bLandOnly || !(pAdjacentPlot->isWater()))
				{
					return true;
				}
			}
		}
	}

	return false;
}


bool CvPlot::isAdjacentTeam(TeamTypes eTeam, bool bLandOnly) const
{
	CvPlot* pAdjacentPlot;
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->getTeam() == eTeam)
			{
				if (!bLandOnly || !(pAdjacentPlot->isWater()))
				{
					return true;
				}
			}
		}
	}

	return false;
}


bool CvPlot::isWithinCultureRange(PlayerTypes ePlayer) const
{
	//allow culture two plots from cities culture
	int iExtraCultureRange = GC.getDefineINT("BUY_PLOT_CULTURE_RANGE");
	for (int iCultureLevel = 0; iCultureLevel < GC.getNumCultureLevelInfos(); ++iCultureLevel)
	{
		for (int iDX = -iExtraCultureRange; iDX <= iExtraCultureRange; iDX++)
		{
			for (int iDY = -iExtraCultureRange; iDY <= iExtraCultureRange; iDY++)
			{
				CvPlot* pLoopPlot = plotXY(getX_INLINE(), getY_INLINE(), iDX, iDY);
				if(pLoopPlot != NULL)
				{
					if (pLoopPlot->isCultureRangeCity(ePlayer, iCultureLevel))
					{
						return true;
					}
				}
			}
		}
	}

// 	for (int iCultureLevel = 0; iCultureLevel < GC.getNumCultureLevelInfos(); ++iCultureLevel)
// 	{
// 		if (isCultureRangeCity(ePlayer, iCultureLevel))
// 		{
// 			return true;
// 		}
// 	}

	return false;
}


int CvPlot::getNumCultureRangeCities(PlayerTypes ePlayer) const
{
	int iCount;
	int iI;

	iCount = 0;

	for (iI = 0; iI < GC.getNumCultureLevelInfos(); ++iI)
	{
		iCount += getCultureRangeCities(ePlayer, iI);
	}

	return iCount;
}


PlayerTypes CvPlot::calculateCulturalOwner() const
{
	if (isForceUnowned())
	{
		return NO_PLAYER;
	}

	int iBestCulture = 0;
	PlayerTypes eBestPlayer = NO_PLAYER;

	//calculate who deserves to own this plot
	for (int iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		if (GET_PLAYER((PlayerTypes)iI).isAlive())
		{
			int iCulture = getCulture((PlayerTypes)iI);

			if (iCulture > 0)
			{
				// Super Forts begin *culture* - modified if statement
				if (isWithinCultureRange((PlayerTypes)iI) || isWithinFortCultureRange((PlayerTypes)iI))
				//if (isWithinCultureRange((PlayerTypes)iI)) - Original Code
				// Super Forts end				
				{
					if (eBestPlayer != NO_PLAYER && GET_PLAYER(eBestPlayer).isNative() && GET_PLAYER((PlayerTypes)iI).getDominateNativeBordersCount() > 0)
					{
						iBestCulture = iCulture;
						eBestPlayer = ((PlayerTypes)iI);
					}
					else if (eBestPlayer == NO_PLAYER || GET_PLAYER(eBestPlayer).getDominateNativeBordersCount() == 0 || !GET_PLAYER((PlayerTypes)iI).isNative())
					{
						if ((iCulture > iBestCulture) || ((iCulture == iBestCulture) && (getOwnerINLINE() == iI)))
						{
							iBestCulture = iCulture;
							eBestPlayer = ((PlayerTypes)iI);
						}
					}
				}
			}
		}
	}

	// See if the presence of a nearby city overrides the player with the highest culture
	if (!isCity())
	{
		if (eBestPlayer != NO_PLAYER)
		{
			int iBestPriority = MAX_INT;
			CvCity* pBestCity = NULL;

			for (int iI = 0; iI < NUM_CITY_PLOTS; ++iI)
			{
				CvPlot* pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

				if (pLoopPlot != NULL)
				{
					CvCity* pLoopCity = pLoopPlot->getPlotCity();

					if (pLoopCity != NULL)
					{
						// Only look at other player's cities
						if (pLoopCity->getOwnerINLINE() != eBestPlayer)
						{
							// don't steal plots from city radius of your teammate
							// don't steal plots from city of European if you're a native
							if (pLoopCity->getTeam() == GET_PLAYER(eBestPlayer).getTeam() || (!pLoopCity->isNative() && GET_PLAYER(eBestPlayer).isNative()))
							{
								if (getCulture(pLoopCity->getOwnerINLINE()) > 0)
								{
									if (isWithinCultureRange(pLoopCity->getOwnerINLINE()))
									{
										int iPriority = GC.getCityPlotPriority()[iI] + 5;
										if ((iPriority < iBestPriority) || ((iPriority == iBestPriority) && (pLoopCity->getOwnerINLINE() == eBestPlayer)))
										{
											iBestPriority = iPriority;
											pBestCity = pLoopCity;
										}
									}
								}
							}
						}
					}
				}
			}

			if (pBestCity != NULL)
			{
				eBestPlayer = pBestCity->getOwnerINLINE();
			}
		}
	}

	// check if we are surrounded by all four sides by the same player
	if (eBestPlayer == NO_PLAYER)
	{
		bool bValid = true;

		for (int iI = 0; iI < NUM_CARDINALDIRECTION_TYPES; ++iI)
		{
			CvPlot* pLoopPlot = plotCardinalDirection(getX_INLINE(), getY_INLINE(), ((CardinalDirectionTypes)iI));

			if (pLoopPlot != NULL)
			{
				if (pLoopPlot->isOwned())
				{
					if (eBestPlayer == NO_PLAYER)
					{
						eBestPlayer = pLoopPlot->getOwnerINLINE();
					}
					else if (eBestPlayer != pLoopPlot->getOwnerINLINE())
					{
						bValid = false;
						break;
					}
				}
				else
				{
					bValid = false;
					break;
				}
			}
		}

		if (!bValid
			// Super Forts begin *culture*
			|| !GET_PLAYER(eBestPlayer).isAlive())
			// Super Forts end
		{
			eBestPlayer = NO_PLAYER;
		}
	}

	return eBestPlayer;
}


void CvPlot::plotAction(PlotUnitFunc func, int iData1, int iData2, PlayerTypes eOwner, TeamTypes eTeam)
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if ((eOwner == NO_PLAYER) || (pLoopUnit->getOwnerINLINE() == eOwner))
		{
			if ((eTeam == NO_TEAM) || (pLoopUnit->getTeam() == eTeam))
			{
				func(pLoopUnit, iData1, iData2);
			}
		}
	}
}


int CvPlot::plotCount(ConstPlotUnitFunc funcA, int iData1A, int iData2A, PlayerTypes eOwner, TeamTypes eTeam, ConstPlotUnitFunc funcB, int iData1B, int iData2B) const
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;
	int iCount;

	iCount = 0;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if ((eOwner == NO_PLAYER) || (pLoopUnit->getOwnerINLINE() == eOwner))
		{
			if ((eTeam == NO_TEAM) || (pLoopUnit->getTeam() == eTeam))
			{
				if ((funcA == NULL) || funcA(pLoopUnit, iData1A, iData2A))
				{
					if ((funcB == NULL) || funcB(pLoopUnit, iData1B, iData2B))
					{
						iCount++;
					}
				}
			}
		}
	}

	return iCount;
}


CvUnit* CvPlot::plotCheck(ConstPlotUnitFunc funcA, int iData1A, int iData2A, PlayerTypes eOwner, TeamTypes eTeam, ConstPlotUnitFunc funcB, int iData1B, int iData2B) const
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if ((eOwner == NO_PLAYER) || (pLoopUnit->getOwnerINLINE() == eOwner))
		{
			if ((eTeam == NO_TEAM) || (pLoopUnit->getTeam() == eTeam))
			{
				if (funcA(pLoopUnit, iData1A, iData2A))
				{
					if ((funcB == NULL) || funcB(pLoopUnit, iData1B, iData2B))
					{
						return pLoopUnit;
					}
				}
			}
		}
	}

	return NULL;
}

bool CvPlot::isOwned() const
{
	return (getOwnerINLINE() != NO_PLAYER);
}

bool CvPlot::isVisible(TeamTypes eTeam, bool bDebug) const
{
	if (bDebug && GC.getGameINLINE().isDebugMode())
	{
		return true;
	}
	else
	{
		if (eTeam == NO_TEAM)
		{
			return false;
		}

		return (getVisibilityCount(eTeam) > 0);
	}
}


bool CvPlot::isActiveVisible(bool bDebug) const
{
	return isVisible(GC.getGameINLINE().getActiveTeam(), bDebug);
}

bool CvPlot::isVisibleToCivTeam() const
{
	int iI;

	for (iI = 0; iI < MAX_TEAMS; ++iI)
	{
		if (GET_TEAM((TeamTypes)iI).isAlive())
		{
			if (isVisible(((TeamTypes)iI), false))
			{
				return true;
			}
		}
	}

	return false;
}


bool CvPlot::isVisibleToWatchingHuman() const
{
	for (int iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		if (GET_PLAYER((PlayerTypes)iI).isAlive())
		{
			if (GET_PLAYER((PlayerTypes)iI).isHuman())
			{
				if (isVisible(GET_PLAYER((PlayerTypes)iI).getTeam(), true))
				{
					return true;
				}
			}
		}
	}

	return false;
}


bool CvPlot::isAdjacentVisible(TeamTypes eTeam, bool bDebug) const
{
	CvPlot* pAdjacentPlot;
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->isVisible(eTeam, bDebug))
			{
				return true;
			}
		}
	}

	return false;
}

bool CvPlot::isAdjacentNonvisible(TeamTypes eTeam) const
{
	CvPlot* pAdjacentPlot;
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (!pAdjacentPlot->isVisible(eTeam, false))
			{
				return true;
			}
		}
	}

	return false;
}


bool CvPlot::isGoody(TeamTypes eTeam) const
{
	if (getImprovementType() == NO_IMPROVEMENT)
	{
		return false;
	}

	return (GC.getImprovementInfo(getImprovementType()).isGoody());
}


bool CvPlot::isRevealedGoody(TeamTypes eTeam) const
{
	if (eTeam == NO_TEAM)
	{
		return isGoody();
	}

	return ((getRevealedImprovementType(eTeam, false) == NO_IMPROVEMENT) ? false : GC.getImprovementInfo(getRevealedImprovementType(eTeam, false)).isGoody());
}


void CvPlot::removeGoody()
{
	setImprovementType(NO_IMPROVEMENT);
}


bool CvPlot::isCity(bool bCheckImprovement, TeamTypes eForTeam) const
{
	if (bCheckImprovement && NO_IMPROVEMENT != getImprovementType())
	{
		if (GC.getImprovementInfo(getImprovementType()).isActsAsCity())
		{
			if (NO_TEAM == eForTeam || (NO_TEAM == getTeam() && GC.getImprovementInfo(getImprovementType()).isOutsideBorders()) || GET_TEAM(eForTeam).isFriendlyTerritory(getTeam()))
			{
				return true;
			}
		}
	}

	return (getPlotCity() != NULL);
}

bool CvPlot::isFriendlyCity(const CvUnit& kUnit, bool bCheckImprovement) const
{
	if (!isCity(bCheckImprovement, kUnit.getTeam()))
	{
		return false;
	}

	if (isVisibleEnemyUnit(&kUnit))
	{
		return false;
	}

	TeamTypes ePlotTeam = getTeam();

	if (NO_TEAM != ePlotTeam)
	{
		TeamTypes eTeam = kUnit.getCombatTeam(ePlotTeam, this);

		if (eTeam == ePlotTeam)
		{
			return true;
		}

		if (eTeam != NO_TEAM && GET_TEAM(ePlotTeam).isOpenBorders(eTeam))
		{
			return true;
		}

		if (!kUnit.isEnemy(ePlotTeam, this) && GET_PLAYER(getOwnerINLINE()).isAlwaysOpenBorders())
		{
			// Military Ships (Frigates, Ship-Of-The-Line, Man-O-Wars) cannot enter native villages
			
			// TAC - Military Cargo Ships (Performance Bugfix) - koma13 - START
			//if (kUnit.getUnitInfo().isHiddenNationality() || kUnit.isOnlyDefensive() || kUnit.getDomainType() != DOMAIN_SEA)
			if (kUnit.getUnitInfo().isHiddenNationality() || kUnit.isOnlyDefensive() || kUnit.getDomainType() != DOMAIN_SEA || kUnit.cargoSpace() > 0)
			// TAC - Military Cargo Ships (Performance Bugfix) - koma13 - END
			{
				return true;
			}
		}
	}

	return false;
}


bool CvPlot::isEnemyCity(const CvUnit& kUnit) const
{
	// Super Forts begin *culture*
	TeamTypes ePlotTeam = getTeam();
	if (isCity(true) && (ePlotTeam != NO_TEAM))
	{
		return kUnit.isEnemy(ePlotTeam, this);
	}
	/* Original Code
	CvCity* pCity = getPlotCity();

	if (pCity != NULL)
	{
		return kUnit.isEnemy(pCity->getTeam(), this);
	} */
	// Super Forts end

	return false;
}

// R&R, ray, Monasteries and Forts - START
bool CvPlot::isFort() const
{
	if (getImprovementType() != NO_IMPROVEMENT && GC.getImprovementInfo(getImprovementType()).isFort())
	{
		return true;
	}

	return false;
}

bool CvPlot::isMonastery() const
{
	if (getImprovementType() != NO_IMPROVEMENT && GC.getImprovementInfo(getImprovementType()).isMonastery())
	{
		return true;
	}

	return false;
}
// R&R, ray, Monasteries and Forts - END


bool CvPlot::isOccupation() const
{
	CvCity* pCity;

	pCity = getPlotCity();

	if (pCity != NULL)
	{
		return pCity->isOccupation();
	}

	return false;
}


bool CvPlot::isBeingWorked() const
{
	CvCity* pWorkingCity = getWorkingCity();

	if (pWorkingCity != NULL)
	{
		return pWorkingCity->isUnitWorkingPlot(this);
	}

	return false;
}


bool CvPlot::isUnit() const
{
	return (getNumUnits() > 0);
}


bool CvPlot::isVisibleEnemyDefender(const CvUnit* pUnit) const
{
	return (plotCheck(PUF_canDefendEnemy, pUnit->getOwnerINLINE(), pUnit->isAlwaysHostile(this), NO_PLAYER, NO_TEAM, PUF_isVisible, pUnit->getOwnerINLINE()) != NULL);
}


CvUnit *CvPlot::getVisibleEnemyDefender(PlayerTypes ePlayer) const
{
	return plotCheck(PUF_canDefendEnemy, ePlayer, false, NO_PLAYER, NO_TEAM, PUF_isVisible, ePlayer);
}


int CvPlot::getNumDefenders(PlayerTypes ePlayer) const
{
	return plotCount(PUF_canDefend, -1, -1, ePlayer);
}


int CvPlot::getNumVisibleEnemyDefenders(const CvUnit* pUnit) const
{
	return plotCount(PUF_canDefendEnemy, pUnit->getOwnerINLINE(), pUnit->isAlwaysHostile(this), NO_PLAYER, NO_TEAM, PUF_isVisible, pUnit->getOwnerINLINE());
}


int CvPlot::getNumVisiblePotentialEnemyDefenders(const CvUnit* pUnit) const
{
	return plotCount(PUF_canDefendPotentialEnemy, pUnit->getOwnerINLINE(), pUnit->isAlwaysHostile(this), NO_PLAYER, NO_TEAM, PUF_isVisible, pUnit->getOwnerINLINE());
}


bool CvPlot::isVisibleEnemyUnit(PlayerTypes ePlayer) const
{
	return (plotCheck(PUF_isEnemy, ePlayer, false, NO_PLAYER, NO_TEAM, PUF_isVisible, ePlayer) != NULL);
}

// R&R, ray, Natives raiding party - START
bool CvPlot::isVisibleEnemyUnit(const CvUnit* pUnit) const
{
	return (plotCheck(PUF_isEnemy, pUnit->getOwnerINLINE(), (pUnit->isAlwaysHostile(this) || (pUnit->AI_getUnitAIState() == UNITAI_STATE_RAIDING_PARTY)), NO_PLAYER, NO_TEAM, PUF_isVisible, pUnit->getOwnerINLINE()) != NULL);
}
// R&R, ray, Natives raiding party - END

bool CvPlot::isVisibleOtherUnit(PlayerTypes ePlayer) const
{
	return (plotCheck(PUF_isOtherTeam, ePlayer, -1, NO_PLAYER, NO_TEAM, PUF_isVisible, ePlayer) != NULL);
}


bool CvPlot::isFighting() const
{
	return (plotCheck(PUF_isFighting) != NULL);
}


bool CvPlot::canHaveFeature(FeatureTypes eFeature) const
{
	CvPlot* pAdjacentPlot;
	int iI;

	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	if (eFeature == NO_FEATURE)
	{
		return true;
	}

	if (getFeatureType() != NO_FEATURE)
	{
		return false;
	}

	if (isPeak())
	{
		return false;
	}

	if (isCity())
	{
		return false;
	}

	if (!(GC.getFeatureInfo(eFeature).isTerrain(getTerrainType())))
	{
		return false;
	}

	if (GC.getFeatureInfo(eFeature).isNoCoast() && isCoastalLand())
	{
		return false;
	}

	if (GC.getFeatureInfo(eFeature).isNoRiver() && isRiver())
	{
		return false;
	}

	if (GC.getFeatureInfo(eFeature).isRequiresFlatlands() && (isHills() || isPeak()))
	{
		return false;
	}

	if (GC.getFeatureInfo(eFeature).isNoAdjacent())
	{
		for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

			if (pAdjacentPlot != NULL)
			{
				if (pAdjacentPlot->getFeatureType() == eFeature)
				{
					return false;
				}
			}
		}
	}

	if (GC.getFeatureInfo(eFeature).isRequiresRiver() && !isRiver())
	{
		return false;
	}

	return true;
}


bool CvPlot::isRoute() const
{
	return (getRouteType() != NO_ROUTE);
}


bool CvPlot::isValidRoute(const CvUnit* pUnit) const
{
	if (isRoute())
	{
		if (!pUnit->isEnemy(getTeam(), this) || pUnit->isEnemyRoute())
		{
			return true;
		}
	}

	return false;
}

bool CvPlot::isValidDomainForLocation(const CvUnit& unit) const
{
	if (unit.getYield() != NO_YIELD)
	{
		CvCity* pCity = getPlotCity();
		if(pCity != NULL)
		{
			if(GET_PLAYER(unit.getOwnerINLINE()).canUnloadYield(pCity->getOwnerINLINE()))
			{
				return true;
			}
		}

		return false;
	}

	if (isValidDomainForAction(unit))
	{
		return true;
	}

	return isCity(true, unit.getTeam());
}


bool CvPlot::isValidDomainForAction(UnitTypes eUnit) const
{
	CvUnitInfo& kUnitInfo = GC.getUnitInfo(eUnit);

	switch (kUnitInfo.getDomainType())
	{
	case DOMAIN_SEA:
		return (isWater() || kUnitInfo.isCanMoveAllTerrain());
		break;

	case DOMAIN_LAND:
	case DOMAIN_IMMOBILE:
		return (!isWater() || kUnitInfo.isCanMoveAllTerrain());
		break;

	default:
		FAssert(false);
		break;
	}

	return false;
}

bool CvPlot::isValidDomainForAction(const CvUnit& unit) const
{
	return isValidDomainForAction(unit.getUnitType());
}


bool CvPlot::isImpassable() const
{
	return m_bImpassable;
}


int CvPlot::getX() const
{
	return m_iX;
}


int CvPlot::getY() const
{
	return m_iY;
}


bool CvPlot::at(int iX, int iY) const
{
	return ((getX_INLINE() == iX) && (getY_INLINE() == iY));
}


int CvPlot::getLatitude() const
{
	int iLatitude;

	if (GC.getMapINLINE().isWrapXINLINE() || !(GC.getMapINLINE().isWrapYINLINE()))
	{
		iLatitude = ((getY_INLINE() * 100) / GC.getMapINLINE().getGridHeightINLINE());
	}
	else
	{
		iLatitude = ((getX_INLINE() * 100) / GC.getMapINLINE().getGridWidthINLINE());
	}

	iLatitude = ((iLatitude * (GC.getMapINLINE().getTopLatitude() - GC.getMapINLINE().getBottomLatitude())) / 100);

	return abs(iLatitude + GC.getMapINLINE().getBottomLatitude());
}


int CvPlot::getFOWIndex() const
{
	return ((((GC.getMapINLINE().getGridHeight() - 1) - getY_INLINE()) * GC.getMapINLINE().getGridWidth() * LANDSCAPE_FOW_RESOLUTION * LANDSCAPE_FOW_RESOLUTION) + (getX_INLINE() * LANDSCAPE_FOW_RESOLUTION));
}


CvArea* CvPlot::area() const
{
	if(m_pPlotArea == NULL)
	{
		m_pPlotArea = GC.getMapINLINE().getArea(getArea());
	}

	return m_pPlotArea;
}


CvArea* CvPlot::waterArea() const
{
	CvArea* pBestArea;
	CvPlot* pAdjacentPlot;
	int iValue;
	int iBestValue;
	int iI;

	if (isWater())
	{
		return area();
	}

	iBestValue = 0;
	pBestArea = NULL;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->isWater())
			{
				iValue = pAdjacentPlot->area()->getNumTiles();

				if (iValue > iBestValue)
				{
					iBestValue = iValue;
					pBestArea = pAdjacentPlot->area();
				}
			}
		}
	}

	return pBestArea;
}

CvArea* CvPlot::secondWaterArea() const
{

	CvArea* pWaterArea = waterArea();
	CvArea* pBestArea;
	CvPlot* pAdjacentPlot;
	int iValue;
	int iBestValue;
	int iI;

	FAssert(!isWater());

	iBestValue = 0;
	pBestArea = NULL;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->isWater() && (pAdjacentPlot->getArea() != pWaterArea->getID()))
			{
				iValue = pAdjacentPlot->area()->getNumTiles();

				if (iValue > iBestValue)
				{
					iBestValue = iValue;
					pBestArea = pAdjacentPlot->area();
				}
			}
		}
	}

	return pBestArea;

}


int CvPlot::getArea() const
{
	return m_iArea;
}


void CvPlot::setArea(int iNewValue)
{
	bool bOldLake;

	if (getArea() != iNewValue)
	{
		bOldLake = isLake();

		if (area() != NULL)
		{
			processArea(area(), -1);
		}

		m_iArea = iNewValue;
		m_pPlotArea = NULL;

		if (area() != NULL)
		{
			processArea(area(), 1);
			updateYield(true);
		}
	}
}


int CvPlot::getFeatureVariety() const
{
	FAssert((getFeatureType() == NO_FEATURE) || (m_iFeatureVariety < GC.getFeatureInfo(getFeatureType()).getArtInfo()->getNumVarieties()));
	FAssert(m_iFeatureVariety >= 0);
	return m_iFeatureVariety;
}


int CvPlot::getOwnershipDuration() const
{
	return m_iOwnershipDuration;
}


bool CvPlot::isOwnershipScore() const
{
	return (getOwnershipDuration() >= GC.getDefineINT("OWNERSHIP_SCORE_DURATION_THRESHOLD"));
}


void CvPlot::setOwnershipDuration(int iNewValue)
{
	bool bOldOwnershipScore;

	if (getOwnershipDuration() != iNewValue)
	{
		bOldOwnershipScore = isOwnershipScore();

		m_iOwnershipDuration = iNewValue;
		FAssert(getOwnershipDuration() >= 0);

		if (bOldOwnershipScore != isOwnershipScore())
		{
			if (isOwned())
			{
				if (!isWater())
				{
					GET_PLAYER(getOwnerINLINE()).changeTotalLandScored((isOwnershipScore()) ? 1 : -1);
				}
			}
		}
	}
}


void CvPlot::changeOwnershipDuration(int iChange)
{
	setOwnershipDuration(getOwnershipDuration() + iChange);
}


int CvPlot::getImprovementDuration() const
{
	return m_iImprovementDuration;
}


void CvPlot::setImprovementDuration(int iNewValue)
{
	m_iImprovementDuration = iNewValue;
	FAssert(getImprovementDuration() >= 0);
}


void CvPlot::changeImprovementDuration(int iChange)
{
	setImprovementDuration(getImprovementDuration() + iChange);
}


int CvPlot::getUpgradeProgress() const
{
	return m_iUpgradeProgress;
}


int CvPlot::getUpgradeTimeLeft(ImprovementTypes eImprovement, PlayerTypes ePlayer) const
{
	int iUpgradeLeft;
	int iUpgradeRate;
	int iTurnsLeft;

	iUpgradeLeft = (GC.getGameINLINE().getImprovementUpgradeTime(eImprovement) - ((getImprovementType() == eImprovement) ? getUpgradeProgress() : 0));

	if (ePlayer == NO_PLAYER)
	{
		return iUpgradeLeft;
	}

	iUpgradeRate = GET_PLAYER(ePlayer).getImprovementUpgradeRate();

	if (iUpgradeRate == 0)
	{
		return iUpgradeLeft;
	}

	iTurnsLeft = (iUpgradeLeft / iUpgradeRate);

	if ((iTurnsLeft * iUpgradeRate) < iUpgradeLeft)
	{
		iTurnsLeft++;
	}

	return std::max(1, iTurnsLeft);
}


void CvPlot::setUpgradeProgress(int iNewValue)
{
	m_iUpgradeProgress = iNewValue;
	FAssert(getUpgradeProgress() >= 0);
}


void CvPlot::changeUpgradeProgress(int iChange)
{
	setUpgradeProgress(getUpgradeProgress() + iChange);
}


int CvPlot::getForceUnownedTimer() const
{
	return m_iForceUnownedTimer;
}


bool CvPlot::isForceUnowned() const
{
	return (getForceUnownedTimer() > 0);
}


void CvPlot::setForceUnownedTimer(int iNewValue)
{
	m_iForceUnownedTimer = iNewValue;
	FAssert(getForceUnownedTimer() >= 0);
}


void CvPlot::changeForceUnownedTimer(int iChange)
{
	setForceUnownedTimer(getForceUnownedTimer() + iChange);
}


int CvPlot::getCityRadiusCount() const
{
	return m_iCityRadiusCount;
}


int CvPlot::isCityRadius() const
{
	return (getCityRadiusCount() > 0);
}


void CvPlot::changeCityRadiusCount(int iChange)
{
	m_iCityRadiusCount = (m_iCityRadiusCount + iChange);
	FAssert(getCityRadiusCount() >= 0);
}


bool CvPlot::isStartingPlot() const
{
	return m_bStartingPlot;
}

void CvPlot::setStartingPlot(bool bNewValue)
{
	m_bStartingPlot = bNewValue;
}


bool CvPlot::isNOfRiver() const
{
	return m_bNOfRiver;
}


void CvPlot::setNOfRiver(bool bNewValue, CardinalDirectionTypes eRiverDir)
{
	CvPlot* pAdjacentPlot;
	int iI;

	if ((isNOfRiver() != bNewValue) || (eRiverDir != m_eRiverWEDirection))
	{
		if (isNOfRiver() != bNewValue)
		{
			m_bNOfRiver = bNewValue;

			updateRiverCrossing();
			updateYield(true);

			for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
			{
				pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

				if (pAdjacentPlot != NULL)
				{
					pAdjacentPlot->updateRiverCrossing();
					pAdjacentPlot->updateYield(true);
				}
			}

			if (area() != NULL)
			{
				area()->changeNumRiverEdges((isNOfRiver()) ? 1 : -1);
			}
		}

		FAssertMsg(eRiverDir == CARDINALDIRECTION_WEST || eRiverDir == CARDINALDIRECTION_EAST || eRiverDir == NO_CARDINALDIRECTION, "invalid parameter");
		m_eRiverWEDirection = eRiverDir;

		updateRiverSymbol(true, true);
	}
}


bool CvPlot::isWOfRiver() const
{
	return m_bWOfRiver;
}


void CvPlot::setWOfRiver(bool bNewValue, CardinalDirectionTypes eRiverDir)
{
	CvPlot* pAdjacentPlot;
	int iI;

	if ((isWOfRiver() != bNewValue) || (eRiverDir != m_eRiverNSDirection))
	{
		if (isWOfRiver() != bNewValue)
		{
			m_bWOfRiver = bNewValue;

			updateRiverCrossing();
			updateYield(true);

			for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
			{
				pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

				if (pAdjacentPlot != NULL)
				{
					pAdjacentPlot->updateRiverCrossing();
					pAdjacentPlot->updateYield(true);
				}
			}

			if (area())
			{
				area()->changeNumRiverEdges((isWOfRiver()) ? 1 : -1);
			}
		}

		FAssertMsg(eRiverDir == CARDINALDIRECTION_NORTH || eRiverDir == CARDINALDIRECTION_SOUTH || eRiverDir == NO_CARDINALDIRECTION, "invalid parameter");
		m_eRiverNSDirection = eRiverDir;

		updateRiverSymbol(true, true);
	}
}


CardinalDirectionTypes CvPlot::getRiverNSDirection() const
{
	return (CardinalDirectionTypes)m_eRiverNSDirection;
}


CardinalDirectionTypes CvPlot::getRiverWEDirection() const
{
	return (CardinalDirectionTypes)m_eRiverWEDirection;
}

EuropeTypes CvPlot::getEurope() const
{
	return (EuropeTypes)m_eEurope;
}

void CvPlot::setEurope(EuropeTypes eEurope)
{
	m_eEurope = (char)eEurope;
	gDLL->getInterfaceIFace()->setDirty(ColoredPlots_DIRTY_BIT, true);
}

//This function determiens what coast (if any) the plot is on.
EuropeTypes CvPlot::getNearestEurope() const
{
	const CvPlot* pCurrentPlot = this;
	
	if (!pCurrentPlot->isWater())
	{
		for (int iDirection = 0; iDirection < NUM_DIRECTION_TYPES; ++iDirection)
		{
			CvPlot* pDirectionPlot = plotDirection(pCurrentPlot->getX_INLINE(), pCurrentPlot->getY_INLINE(), (DirectionTypes)iDirection);
			if (pDirectionPlot != NULL)
			{
				if (pDirectionPlot->isWater() && pDirectionPlot->isEuropeAccessable())
				{
					pCurrentPlot = pDirectionPlot;
					break;
				}
			}
		}
	}

	int iHack = 0;
	while (iHack++ < 1000)
	{
		const CvPlot* pBestPlot = NULL;
		for (int iDirection = 0; iDirection < NUM_DIRECTION_TYPES; ++iDirection)
		{
			CvPlot* pDirectionPlot = plotDirection(pCurrentPlot->getX_INLINE(), pCurrentPlot->getY_INLINE(), (DirectionTypes)iDirection);
			if (pDirectionPlot != NULL)
			{
				if (pDirectionPlot->isWater() && !pDirectionPlot->isImpassable())
				{
					if (pDirectionPlot->isEurope())
					{
						return pDirectionPlot->getEurope();
					}

					if (pDirectionPlot->getDistanceToOcean() < pCurrentPlot->getDistanceToOcean())
					{
						pBestPlot = pDirectionPlot;
						break;
					}
				}
			}
		}
		if (pBestPlot == NULL)
		{
			return NO_EUROPE;
		}
		pCurrentPlot = pBestPlot;
	}
	
	FAssert(false);
	return NO_EUROPE;
}

bool CvPlot::isEuropeAccessable() const
{
	return getDistanceToOcean() != MAX_SHORT;	
}

// This function finds an *inland* corner of this plot at which to place a river.
// It then returns the plot with that corner at its SE.

CvPlot* CvPlot::getInlandCorner() const
{
	CvPlot* pRiverPlot = NULL; // will be a plot through whose SE corner we want the river to run

	std::vector<int> aiShuffle(4);
	GC.getGameINLINE().getMapRand().shuffleSequence(aiShuffle, NULL);

	for (int iI = 0; iI < 4; ++iI)
	{
		switch (aiShuffle[iI])
		{
		case 0:
			pRiverPlot = GC.getMapINLINE().plotSorenINLINE(getX_INLINE(), getY_INLINE()); break;
		case 1:
			pRiverPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_NORTH); break;
		case 2:
			pRiverPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_NORTHWEST); break;
		case 3:
			pRiverPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_WEST); break;
		}
		if (pRiverPlot != NULL && !pRiverPlot->hasCoastAtSECorner())
		{
			break;
		}
		else
		{
			pRiverPlot = NULL;
		}
	}

	return pRiverPlot;
}


bool CvPlot::hasCoastAtSECorner() const
{
	CvPlot* pAdjacentPlot;

	if (isWater())
	{
		return true;
	}

	pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_EAST);
	if (pAdjacentPlot != NULL && pAdjacentPlot->isWater())
	{
		return true;
	}

	pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_SOUTHEAST);
	if (pAdjacentPlot != NULL && pAdjacentPlot->isWater())
	{
		return true;
	}

	pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_SOUTH);
	if (pAdjacentPlot != NULL && pAdjacentPlot->isWater())
	{
		return true;
	}

	return false;
}


bool CvPlot::isPotentialCityWork() const
{
	return m_bPotentialCityWork;
}

bool CvPlot::isPotentialCityWorkForArea(CvArea* pArea) const
{
	PROFILE_FUNC();

	CvPlot* pLoopPlot;
	int iI;

	for (iI = 0; iI < NUM_CITY_PLOTS; ++iI)
	{
		pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

		if (pLoopPlot != NULL)
		{
			if (!pLoopPlot->isWater())
			{
				if (pLoopPlot->area() == pArea)
				{
					return true;
				}
			}
		}
	}

	return false;
}

void CvPlot::updatePotentialCityWork()
{
	PROFILE_FUNC();

	CvPlot* pLoopPlot;
	bool bValid;
	int iI;

	bValid = false;

	for (iI = 0; iI < NUM_CITY_PLOTS; ++iI)
	{
		pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

		if (pLoopPlot != NULL)
		{
			if (!(pLoopPlot->isWater()))
			{
				bValid = true;
				break;
			}
		}
	}

	if (isPotentialCityWork() != bValid)
	{
		m_bPotentialCityWork = bValid;

		updateYield(true);
	}
}


bool CvPlot::isShowCitySymbols() const
{
	return m_bShowCitySymbols;
}


void CvPlot::updateShowCitySymbols()
{
	CvCity* pLoopCity;
	CvPlot* pLoopPlot;
	bool bNewShowCitySymbols;
	int iI;

	bNewShowCitySymbols = false;

	for (iI = 0; iI < NUM_CITY_PLOTS; ++iI)
	{
		pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

		if (pLoopPlot != NULL)
		{
			pLoopCity = pLoopPlot->getPlotCity();

			if (pLoopCity != NULL)
			{
				if (pLoopCity->isCitySelected() && gDLL->getInterfaceIFace()->isCityScreenUp())
				{
					if (pLoopCity->canWork(this))
					{
						bNewShowCitySymbols = true;
						break;
					}
				}
			}
		}
	}

	if (isShowCitySymbols() != bNewShowCitySymbols)
	{
		m_bShowCitySymbols = bNewShowCitySymbols;

		updateSymbols();
	}
}


bool CvPlot::isFlagDirty() const
{
	return m_bFlagDirty;
}


void CvPlot::setFlagDirty(bool bNewValue)
{
	m_bFlagDirty = bNewValue;
}


PlayerTypes CvPlot::getOwner() const
{
	return getOwnerINLINE();
}


void CvPlot::setOwner(PlayerTypes eNewValue, bool bCheckUnits)
{
	PROFILE_FUNC();

	if (getOwnerINLINE() != eNewValue)
	{
		FAssert(getPlotCity() == NULL);

		GC.getGameINLINE().addReplayMessage(REPLAY_MESSAGE_PLOT_OWNER_CHANGE, eNewValue, (char*)NULL, getX_INLINE(), getY_INLINE());

		setOwnershipDuration(0);

		if (isOwned())
		{
			changeAdjacentSight(getTeam(), GC.getPLOT_VISIBILITY_RANGE(), false, NULL);

			if (area())
			{
				area()->changeNumOwnedTiles(-1);
			}
			GC.getMapINLINE().changeOwnedPlots(-1);

			if (!isWater())
			{
				GET_PLAYER(getOwnerINLINE()).changeTotalLand(-1);
				GET_TEAM(getTeam()).changeTotalLand(-1);

				if (isOwnershipScore())
				{
					GET_PLAYER(getOwnerINLINE()).changeTotalLandScored(-1);
				}
			}

			if (getImprovementType() != NO_IMPROVEMENT)
			{
				GET_PLAYER(getOwnerINLINE()).changeImprovementCount(getImprovementType(), -1);
				// Super Forts begin *culture*
				if (GC.getImprovementInfo(getImprovementType()).isActsAsCity())
				{
					changeCultureRangeFortsWithinRange(getOwnerINLINE(), -1, GC.getImprovementInfo(getImprovementType()).getCultureRange(), false);
				}
				// Super Forts end
			}
		}

		m_eOwner = eNewValue;

		setWorkingCityOverride(NULL);
		updateWorkingCity();

		if (isOwned())
		{
			changeAdjacentSight(getTeam(), GC.getPLOT_VISIBILITY_RANGE(), true, NULL);

			if (area())
			{
				area()->changeNumOwnedTiles(1);
			}
			GC.getMapINLINE().changeOwnedPlots(1);

			if (!isWater())
			{
				GET_PLAYER(getOwnerINLINE()).changeTotalLand(1);
				GET_TEAM(getTeam()).changeTotalLand(1);

				if (isOwnershipScore())
				{
					GET_PLAYER(getOwnerINLINE()).changeTotalLandScored(1);
				}
			}

			if (getImprovementType() != NO_IMPROVEMENT)
			{
				GET_PLAYER(getOwnerINLINE()).changeImprovementCount(getImprovementType(), 1);
				// Super Forts begin *culture*
				if (GC.getImprovementInfo(getImprovementType()).isActsAsCity())
				{
					changeCultureRangeFortsWithinRange(getOwnerINLINE(), 1, GC.getImprovementInfo(getImprovementType()).getCultureRange(), true);
				}
				// Super Forts end
			}
		}

		for (int iI = 0; iI < MAX_TEAMS; ++iI)
		{
			if (GET_TEAM((TeamTypes)iI).isAlive())
			{
				updateRevealedOwner((TeamTypes)iI);
			}
		}

		updateYield(true);

		if (bCheckUnits)
		{
			verifyUnitValidPlot();
		}

		if (isOwned())
		{
			if (isGoody())
			{
				GET_PLAYER(getOwnerINLINE()).doGoody(this, NULL);
			}
		}

		if (GC.getGameINLINE().isDebugMode())
		{
			updateMinimapColor();

			gDLL->getInterfaceIFace()->setDirty(GlobeLayer_DIRTY_BIT, true);

			gDLL->getEngineIFace()->SetDirty(CultureBorders_DIRTY_BIT, true);
		}

		updateSymbols();

		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}


PlotTypes CvPlot::getPlotType() const
{
	return (PlotTypes)m_ePlotType;
}


bool CvPlot::isWater() const
{
	return (getPlotType() == PLOT_OCEAN);
}

bool CvPlot::isEurope() const
{
	return (getEurope() != NO_EUROPE);
}


bool CvPlot::isFlatlands() const
{
	return (getPlotType() == PLOT_LAND);
}


bool CvPlot::isHills() const
{
	return (getPlotType() == PLOT_HILLS);
}


bool CvPlot::isPeak() const
{
	return (getPlotType() == PLOT_PEAK);
}


void CvPlot::setPlotType(PlotTypes eNewValue, bool bRecalculate, bool bRebuildGraphics)
{
	CvArea* pNewArea;
	CvArea* pCurrArea;
	CvArea* pLastArea;
	CvPlot* pLoopPlot;
	bool bWasWater;
	bool bRecalculateAreas;
	int iAreaCount;
	int iI;

	if (getPlotType() != eNewValue)
	{
		if ((getPlotType() == PLOT_OCEAN) || (eNewValue == PLOT_OCEAN))
		{
			erase();
		}

		bWasWater = isWater();

		updateSeeFromSight(false);

		m_ePlotType = eNewValue;

		updateYield(true);

		updateSeeFromSight(true);

		if ((getTerrainType() == NO_TERRAIN) || (GC.getTerrainInfo(getTerrainType()).isWater() != isWater()))
		{
			if (isWater())
			{
				if (isAdjacentToLand())
				{
					setTerrainType(((TerrainTypes)(GC.getDefineINT("SHALLOW_WATER_TERRAIN"))), bRecalculate, bRebuildGraphics);
				}
				else
				{
					setTerrainType(((TerrainTypes)(GC.getDefineINT("DEEP_WATER_TERRAIN"))), bRecalculate, bRebuildGraphics);
				}
			}
			else
			{
				setTerrainType(((TerrainTypes)(GC.getDefineINT("LAND_TERRAIN"))), bRecalculate, bRebuildGraphics);
			}
		}

		GC.getMapINLINE().resetPathDistance();

		if (bWasWater != isWater())
		{
			if (bRecalculate)
			{
				for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
				{
					pLoopPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

					if (pLoopPlot != NULL)
					{
						if (pLoopPlot->isWater())
						{
							if (pLoopPlot->isAdjacentToLand())
							{
								pLoopPlot->setTerrainType(((TerrainTypes)(GC.getDefineINT("SHALLOW_WATER_TERRAIN"))), bRecalculate, bRebuildGraphics);
							}
							else
							{
								pLoopPlot->setTerrainType(((TerrainTypes)(GC.getDefineINT("DEEP_WATER_TERRAIN"))), bRecalculate, bRebuildGraphics);
							}
						}
					}
				}
			}

			for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
			{
				pLoopPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

				if (pLoopPlot != NULL)
				{
					pLoopPlot->updateYield(true);
				}
			}

			for (iI = 0; iI < NUM_CITY_PLOTS; ++iI)
			{
				pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

				if (pLoopPlot != NULL)
				{
					pLoopPlot->updatePotentialCityWork();
				}
			}

			GC.getMapINLINE().changeLandPlots((isWater()) ? -1 : 1);

			if (getBonusType() != NO_BONUS)
			{
				GC.getMapINLINE().changeNumBonusesOnLand(getBonusType(), ((isWater()) ? -1 : 1));
			}

			if (isOwned())
			{
				GET_PLAYER(getOwnerINLINE()).changeTotalLand((isWater()) ? -1 : 1);
				GET_TEAM(getTeam()).changeTotalLand((isWater()) ? -1 : 1);
			}

			if (bRecalculate)
			{
				pNewArea = NULL;
				bRecalculateAreas = false;

				// XXX might want to change this if we allow diagonal water movement...
				if (isWater())
				{
					for (iI = 0; iI < NUM_CARDINALDIRECTION_TYPES; ++iI)
					{
						pLoopPlot = plotCardinalDirection(getX_INLINE(), getY_INLINE(), ((CardinalDirectionTypes)iI));

						if (pLoopPlot != NULL)
						{
							if (pLoopPlot->area()->isWater())
							{
								if (pNewArea == NULL)
								{
									pNewArea = pLoopPlot->area();
								}
								else if (pNewArea != pLoopPlot->area())
								{
									bRecalculateAreas = true;
									break;
								}
							}
						}
					}
				}
				else
				{
					for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
					{
						pLoopPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

						if (pLoopPlot != NULL)
						{
							if (!(pLoopPlot->area()->isWater()))
							{
								if (pNewArea == NULL)
								{
									pNewArea = pLoopPlot->area();
								}
								else if (pNewArea != pLoopPlot->area())
								{
									bRecalculateAreas = true;
									break;
								}
							}
						}
					}
				}

				if (!bRecalculateAreas)
				{
					pLoopPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)(NUM_DIRECTION_TYPES - 1)));

					if (pLoopPlot != NULL)
					{
						pLastArea = pLoopPlot->area();
					}
					else
					{
						pLastArea = NULL;
					}

					iAreaCount = 0;

					for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
					{
						pLoopPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

						if (pLoopPlot != NULL)
						{
							pCurrArea = pLoopPlot->area();
						}
						else
						{
							pCurrArea = NULL;
						}

						if (pCurrArea != pLastArea)
						{
							iAreaCount++;
						}

						pLastArea = pCurrArea;
					}

					if (iAreaCount > 2)
					{
						bRecalculateAreas = true;
					}
				}

				if (bRecalculateAreas)
				{
					GC.getMapINLINE().recalculateAreas();
				}
				else
				{
					setArea(FFreeList::INVALID_INDEX);

					if ((area() != NULL) && (area()->getNumTiles() == 1))
					{
						GC.getMapINLINE().deleteArea(getArea());
					}

					if (pNewArea == NULL)
					{
						pNewArea = GC.getMapINLINE().addArea();
						pNewArea->init(pNewArea->getID(), isWater());
					}

					setArea(pNewArea->getID());
				}
			}
		}

		if (bRebuildGraphics && GC.IsGraphicsInitialized())
		{
			//Update terrain graphical
			gDLL->getEngineIFace()->RebuildPlot(getX_INLINE(), getY_INLINE(), true, true);
			//gDLL->getEngineIFace()->SetDirty(MinimapTexture_DIRTY_BIT, true); //minimap does a partial update
			//gDLL->getEngineIFace()->SetDirty(GlobeTexture_DIRTY_BIT, true);

			updateFeatureSymbol();
			setLayoutDirty(true);
			updateRouteSymbol(false, true);
			updateRiverSymbol(false, true);
		}
		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}


TerrainTypes CvPlot::getTerrainType() const
{
	return (TerrainTypes)m_eTerrainType;
}


void CvPlot::setTerrainType(TerrainTypes eNewValue, bool bRecalculate, bool bRebuildGraphics)
{
	bool bUpdateSight;

	if (getTerrainType() != eNewValue)
	{
		if ((getTerrainType() != NO_TERRAIN) &&
			  (eNewValue != NO_TERRAIN) &&
			  ((GC.getTerrainInfo(getTerrainType()).getSeeFromLevel() != GC.getTerrainInfo(eNewValue).getSeeFromLevel()) ||
				 (GC.getTerrainInfo(getTerrainType()).getSeeThroughLevel() != GC.getTerrainInfo(eNewValue).getSeeThroughLevel())))
		{
			bUpdateSight = true;
		}
		else
		{
			bUpdateSight = false;
		}

		if (bUpdateSight)
		{
			updateSeeFromSight(false);
		}

		m_eTerrainType = eNewValue;
		updateImpassable();

		updateYield(true);

		if (bUpdateSight)
		{
			updateSeeFromSight(true);
		}

		if (bRebuildGraphics && GC.IsGraphicsInitialized())
		{
			//Update terrain graphics
			gDLL->getEngineIFace()->RebuildPlot(getX_INLINE(), getY_INLINE(),false,true);
			//gDLL->getEngineIFace()->SetDirty(MinimapTexture_DIRTY_BIT, true); //minimap does a partial update
			//gDLL->getEngineIFace()->SetDirty(GlobeTexture_DIRTY_BIT, true);
		}

		if (GC.getTerrainInfo(getTerrainType()).isWater() != isWater())
		{
			setPlotType(((GC.getTerrainInfo(getTerrainType()).isWater()) ? PLOT_OCEAN : PLOT_LAND), bRecalculate, bRebuildGraphics);
		}
		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}


FeatureTypes CvPlot::getFeatureType() const
{
	return (FeatureTypes)m_eFeatureType;
}


void CvPlot::setFeatureType(FeatureTypes eNewValue, int iVariety)
{
	FeatureTypes eOldFeature;
	bool bUpdateSight;

	eOldFeature = getFeatureType();

	if (eNewValue != NO_FEATURE)
	{
		if (iVariety == -1)
		{
			iVariety = ((GC.getFeatureInfo(eNewValue).getArtInfo()->getNumVarieties() * ((getLatitude() * 9) / 8)) / 90);
		}

		iVariety = range(iVariety, 0, (GC.getFeatureInfo(eNewValue).getArtInfo()->getNumVarieties() - 1));
	}
	else
	{
		iVariety = 0;
	}

	if ((eOldFeature != eNewValue) || (m_iFeatureVariety != iVariety))
	{
		if ((eOldFeature == NO_FEATURE) ||
			  (eNewValue == NO_FEATURE) ||
			  (GC.getFeatureInfo(eOldFeature).getSeeThroughChange() != GC.getFeatureInfo(eNewValue).getSeeThroughChange()))
		{
			bUpdateSight = true;
		}
		else
		{
			bUpdateSight = false;
		}

		if (bUpdateSight)
		{
			updateSeeFromSight(false);
		}

		m_eFeatureType = eNewValue;
		m_iFeatureVariety = iVariety;
		updateImpassable();

		updateYield(true);

		if (bUpdateSight)
		{
			updateSeeFromSight(true);
		}

		updateFeatureSymbol();

		if (getFeatureType() == NO_FEATURE)
		{
			if (getImprovementType() != NO_IMPROVEMENT)
			{
				if (GC.getImprovementInfo(getImprovementType()).isRequiresFeature())
				{
					setImprovementType(NO_IMPROVEMENT);
				}
			}
		}
		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}

BonusTypes CvPlot::getBonusType() const
{
	return (BonusTypes)m_eBonusType;
}

void CvPlot::setBonusType(BonusTypes eNewValue)
{
	if (getBonusType() != eNewValue)
	{
		if (getBonusType() != NO_BONUS)
		{
			if (area())
			{
				area()->changeNumBonuses(getBonusType(), -1);
			}
			GC.getMapINLINE().changeNumBonuses(getBonusType(), -1);

			if (!isWater())
			{
				GC.getMapINLINE().changeNumBonusesOnLand(getBonusType(), -1);
			}
		}

		m_eBonusType = eNewValue;

		if (getBonusType() != NO_BONUS)
		{
			if (area())
			{
				area()->changeNumBonuses(getBonusType(), 1);
			}
			GC.getMapINLINE().changeNumBonuses(getBonusType(), 1);

			if (!isWater())
			{
				GC.getMapINLINE().changeNumBonusesOnLand(getBonusType(), 1);
			}
		}

		updateYield(true);

		setLayoutDirty(true);
		if(getPlotCity() != NULL)
		{
			getPlotCity()->setLayoutDirty(true);
		}

		gDLL->getInterfaceIFace()->setDirty(GlobeLayer_DIRTY_BIT, true);

		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}


ImprovementTypes CvPlot::getImprovementType() const
{
	return (ImprovementTypes)m_eImprovementType;
}


void CvPlot::setImprovementType(ImprovementTypes eNewValue)
{
	ImprovementTypes eOldImprovement = getImprovementType();

	if (getImprovementType() != eNewValue)
	{
		if (getImprovementType() != NO_IMPROVEMENT)
		{
			if (area())
			{
				area()->changeNumImprovements(getImprovementType(), -1);
			}
			if (isOwned())
			{
				GET_PLAYER(getOwnerINLINE()).changeImprovementCount(getImprovementType(), -1);
				// Super Forts begin *culture*
				if (GC.getImprovementInfo(getImprovementType()).isActsAsCity())
				{
					changeCultureRangeFortsWithinRange(getOwnerINLINE(), -1, GC.getImprovementInfo(getImprovementType()).getCultureRange(), true);
				}
				// Super Forts end
			}
		}

		// Super Forts begin *vision*
		updateSight(false);
		// Super Forts end

		m_eImprovementType = eNewValue;

		if (getImprovementType() == NO_IMPROVEMENT)
		{
			setImprovementDuration(0);
		}

		setUpgradeProgress(0);

		for (int iI = 0; iI < MAX_TEAMS; ++iI)
		{
			if (GET_TEAM((TeamTypes)iI).isAlive())
			{
				if (isVisible((TeamTypes)iI, false))
				{
					setRevealedImprovementType((TeamTypes)iI, getImprovementType());
				}
			}
		}

		if (getImprovementType() != NO_IMPROVEMENT)
		{
			if (area())
			{
				area()->changeNumImprovements(getImprovementType(), 1);
			}
			if (isOwned())
			{
				GET_PLAYER(getOwnerINLINE()).changeImprovementCount(getImprovementType(), 1);
				// Super Forts begin *culture*
				if (GC.getImprovementInfo(getImprovementType()).isActsAsCity())
				{
					changeCultureRangeFortsWithinRange(getOwnerINLINE(), 1, GC.getImprovementInfo(getImprovementType()).getCultureRange(), true);
				}
				// Super Forts end
			}
		}

		
		updateSight(true); // Super Forts *vision*
		

		updateYield(true);

		if (NO_FEATURE != eOldImprovement && GC.getImprovementInfo(eOldImprovement).isActsAsCity())
		{
			verifyUnitValidPlot();
		}

		if (GC.getGameINLINE().isDebugMode())
		{
			setLayoutDirty(true);
		}

		if (getImprovementType() != NO_IMPROVEMENT)
		{
			gDLL->getEventReporterIFace()->improvementBuilt(getImprovementType(), getX_INLINE(), getY_INLINE());
		}

		if (getImprovementType() == NO_IMPROVEMENT)
		{
			gDLL->getEventReporterIFace()->improvementDestroyed(eOldImprovement, getOwnerINLINE(), getX_INLINE(), getY_INLINE());
		}

		gDLL->getInterfaceIFace()->setDirty(CitizenButtons_DIRTY_BIT, true);

		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}


RouteTypes CvPlot::getRouteType() const
{
	return (RouteTypes)m_eRouteType;
}


void CvPlot::setRouteType(RouteTypes eNewValue)
{
	bool bOldRoute;
	int iI;

	if (getRouteType() != eNewValue)
	{
		bOldRoute = isRoute(); // XXX is this right???

		m_eRouteType = eNewValue;

		for (iI = 0; iI < MAX_TEAMS; ++iI)
		{
			if (GET_TEAM((TeamTypes)iI).isAlive())
			{
				if (isVisible((TeamTypes)iI, false))
				{
					setRevealedRouteType((TeamTypes)iI, getRouteType());
				}
			}
		}

		updateYield(true);

		if (GC.getGameINLINE().isDebugMode())
		{
			updateRouteSymbol(true, true);
		}

		if (getRouteType() != NO_ROUTE)
		{
			gDLL->getEventReporterIFace()->routeBuilt(getRouteType(), getX_INLINE(), getY_INLINE());
		}

		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}


void CvPlot::updateCityRoute()
{
	RouteTypes eCityRoute;

	if (isCity())
	{
		FAssertMsg(isOwned(), "isOwned is expected to be true");

		eCityRoute = GET_PLAYER(getOwnerINLINE()).getBestRoute();

		if (eCityRoute == NO_ROUTE)
		{
			eCityRoute = ((RouteTypes)(GC.getDefineINT("INITIAL_CITY_ROUTE_TYPE")));
		}

		setRouteType(eCityRoute);
	}
}


CvCity* CvPlot::getPlotCity() const
{
	return getCity(m_plotCity);
}


void CvPlot::setPlotCity(CvCity* pNewValue)
{
	CvPlot* pLoopPlot;
	int iI;

	if (getPlotCity() != pNewValue)
	{
		if (isCity())
		{
			for (iI = 0; iI < NUM_CITY_PLOTS; ++iI)
			{
				pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

				if (pLoopPlot != NULL)
				{
					pLoopPlot->changeCityRadiusCount(-1);
					pLoopPlot->changePlayerCityRadiusCount(getPlotCity()->getOwnerINLINE(), -1);
				}
			}
		}

		if (pNewValue != NULL)
		{
			m_plotCity = pNewValue->getIDInfo();
		}
		else
		{
			m_plotCity.reset();
		}

		if (isCity())
		{
			for (iI = 0; iI < NUM_CITY_PLOTS; ++iI)
			{
				pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

				if (pLoopPlot != NULL)
				{
					pLoopPlot->changeCityRadiusCount(1);
					pLoopPlot->changePlayerCityRadiusCount(getPlotCity()->getOwnerINLINE(), 1);
				}
			}
		}

		updateYield(false);

		updateMinimapColor();

		// CvPlot::hasYield cache - start - Nightinggale
		setYieldCache();
		// CvPlot::hasYield cache - end - Nightinggale
	}
}


CvCity* CvPlot::getWorkingCity() const
{
	return getCity(m_workingCity);
}


void CvPlot::updateWorkingCity()
{
	CvCity* pOldWorkingCity;
	CvCity* pLoopCity;
	CvCity* pBestCity;
	CvPlot* pLoopPlot;
	int iBestPlot;
	int iI;

	pBestCity = getPlotCity();

	if (pBestCity == NULL)
	{
		pBestCity = getWorkingCityOverride();
		FAssertMsg((pBestCity == NULL) || (pBestCity->getOwnerINLINE() == getOwnerINLINE()), "pBest city is expected to either be NULL or the current plot instance's");
	}

	if ((pBestCity == NULL) && isOwned())
	{
		iBestPlot = 0;

		for (iI = 0; iI < NUM_CITY_PLOTS; ++iI)
		{
			pLoopPlot = plotCity(getX_INLINE(), getY_INLINE(), iI);

			if (pLoopPlot != NULL)
			{
				pLoopCity = pLoopPlot->getPlotCity();

				if (pLoopCity != NULL)
				{
					if (pLoopCity->getOwnerINLINE() == getOwnerINLINE())
					{
						// XXX use getGameTurnAcquired() instead???
						if ((pBestCity == NULL) || 
							(!pLoopCity->isHuman() && (pLoopCity->getGameTurnAcquired() < pBestCity->getGameTurnAcquired())) ||
							  (GC.getCityPlotPriority()[iI] < GC.getCityPlotPriority()[iBestPlot]) ||
							  ((GC.getCityPlotPriority()[iI] == GC.getCityPlotPriority()[iBestPlot]) &&
							   ((pLoopCity->getGameTurnFounded() < pBestCity->getGameTurnFounded()) ||
							    ((pLoopCity->getGameTurnFounded() == pBestCity->getGameTurnFounded()) &&
							     (pLoopCity->getID() < pBestCity->getID())))))
						{
							iBestPlot = iI;
							pBestCity = pLoopCity;
						}
					}
				}
			}
		}
	}

	pOldWorkingCity = getWorkingCity();

	if (pOldWorkingCity != pBestCity)
	{
		if (pOldWorkingCity != NULL)
		{
			CvUnit* pUnit = pOldWorkingCity->getUnitWorkingPlot(this);
			if (pUnit != NULL)
			{
				pUnit->setColonistLocked(false);
			}
			pOldWorkingCity->clearUnitWorkingPlot(this);
		}

		if (pBestCity != NULL)
		{
			FAssertMsg(isOwned(), "isOwned is expected to be true");
			FAssertMsg(!isBeingWorked(), "isBeingWorked did not return false as expected");
			m_workingCity = pBestCity->getIDInfo();
		}
		else
		{
			m_workingCity.reset();
		}

		if (pOldWorkingCity != NULL)
		{
			pOldWorkingCity->AI_setAssignWorkDirty(true);
		}
		if (getWorkingCity() != NULL)
		{
			getWorkingCity()->AI_setAssignWorkDirty(true);
		}

		updateYield(true);
		setYieldCache();

		updateFog();
		updateShowCitySymbols();
	}
}


CvCity* CvPlot::getWorkingCityOverride() const
{
	return getCity(m_workingCityOverride);
}


void CvPlot::setWorkingCityOverride( const CvCity* pNewValue)
{
	if (getWorkingCityOverride() != pNewValue)
	{
		if (pNewValue != NULL)
		{
			FAssertMsg(pNewValue->getOwnerINLINE() == getOwnerINLINE(), "Argument city pNewValue's owner is expected to be the same as the current instance");
			m_workingCityOverride = pNewValue->getIDInfo();
		}
		else
		{
			m_workingCityOverride.reset();
		}

		updateWorkingCity();
	}
}


int CvPlot::getRiverID() const
{
	return m_iRiverID;
}


void CvPlot::setRiverID(int iNewValue)
{
	m_iRiverID = iNewValue;
}


int CvPlot::getMinOriginalStartDist() const
{
	return m_iMinOriginalStartDist;
}


void CvPlot::setMinOriginalStartDist(int iNewValue)
{
	m_iMinOriginalStartDist = iNewValue;
}


int CvPlot::getRiverCrossingCount() const
{
	return m_iRiverCrossingCount;
}


void CvPlot::changeRiverCrossingCount(int iChange)
{
	m_iRiverCrossingCount = (m_iRiverCrossingCount + iChange);
	FAssert(getRiverCrossingCount() >= 0);
}


short* CvPlot::getYield()
{
	return m_aiYield;
}


int CvPlot::getYield(YieldTypes eIndex) const
{
	FAssertMsg(eIndex >= 0, "eIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < NUM_YIELD_TYPES, "eIndex is expected to be within maximum bounds (invalid Index)");
	return m_aiYield[eIndex];
}

// TAC - AI Improved Naval AI - koma13 - START
int CvPlot::getDangerMap(PlayerTypes eIndex) const
{
	FAssertMsg(eIndex >= 0, "eIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "eIndex is expected to be within maximum bounds (invalid Index)");
	return m_aiDangerMap[eIndex];
}

void CvPlot::setDangerMap(PlayerTypes eIndex, int iNewValue)
{
	FAssertMsg(eIndex >= 0, "eIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "eIndex is expected to be within maximum bounds (invalid Index)");

	if (m_aiDangerMap[eIndex] != iNewValue)
	{
		m_aiDangerMap[eIndex] = iNewValue;
	}

	FAssert(m_aiDangerMap[eIndex] >= 0);
}
// TAC - AI Improved Naval AI - koma13 - END

int CvPlot::calculateNatureYield(YieldTypes eYield, TeamTypes eTeam, bool bIgnoreFeature) const
{
	int iYield = 0;

	if (isImpassable())
	{
		return 0;
	}

	FAssertMsg(getTerrainType() != NO_TERRAIN, "TerrainType is not assigned a valid value");

	if (isPeak())
	{
		// R&R, ray, fix with Yields on Peaks
		if (GC.getYieldInfo(eYield).getPeakChange() > 0)
		{
			iYield += GC.getTerrainInfo(getTerrainType()).getYield(eYield);
		}
		iYield += GC.getYieldInfo(eYield).getPeakChange();
	}
	else
	{
		iYield += GC.getTerrainInfo(getTerrainType()).getYield(eYield);
	}

	if (isHills())
	{
		iYield += GC.getYieldInfo(eYield).getHillsChange();
	}

	if (isLake())
	{
		iYield += GC.getYieldInfo(eYield).getLakeChange();
	}

	BonusTypes eBonus = getBonusType();
	FeatureTypes eFeature = bIgnoreFeature ? NO_FEATURE : getFeatureType();
	if (eFeature == NO_FEATURE)
	{
		if (eBonus != NO_BONUS && GC.getBonusInfo(eBonus).isTerrain(getTerrainType()))
		{
			iYield += GC.getBonusInfo(eBonus).getYieldChange(eYield);
		}
	}
	else
	{
		iYield += GC.getFeatureInfo(eFeature).getYieldChange(eYield);

		if (eBonus != NO_BONUS && GC.getBonusInfo(eBonus).isFeature(eFeature) && GC.getBonusInfo(eBonus).isFeatureTerrain(getTerrainType()))
		{
			iYield += GC.getBonusInfo(eBonus).getYieldChange(eYield);
		}
	}

	if (isRiver())
	{
		if (iYield > 0)
		{
			iYield += ((bIgnoreFeature || (getFeatureType() == NO_FEATURE)) ? GC.getTerrainInfo(getTerrainType()).getRiverYieldIncrease(eYield) : GC.getFeatureInfo(getFeatureType()).getRiverYieldIncrease(eYield));
		}
	}

	return std::max(0, iYield);
}


int CvPlot::calculateBestNatureYield(YieldTypes eIndex, TeamTypes eTeam) const
{
	return std::max(calculateNatureYield(eIndex, eTeam, false), calculateNatureYield(eIndex, eTeam, true));
}


int CvPlot::calculateTotalBestNatureYield(TeamTypes eTeam) const
{
	int iTotalYield = 0;
	for (int i = 0; i < NUM_YIELD_TYPES; ++i)
	{
		iTotalYield += calculateBestNatureYield((YieldTypes)i, eTeam);
	}

	return iTotalYield;
}


int CvPlot::calculateImprovementYieldChange(ImprovementTypes eImprovement, YieldTypes eYield, PlayerTypes ePlayer, bool bOptimal) const
{
	PROFILE_FUNC();

	BonusTypes eBonus;
	int iBestYield;
	int iYield;
	int iI;

	iYield = 0;

	if (isRiverSide())
	{
		iYield += GC.getImprovementInfo(eImprovement).getRiverSideYieldChange(eYield);
	}

	if (isHills())
	{
		iYield += GC.getImprovementInfo(eImprovement).getHillsYieldChange(eYield);
	}

	if (bOptimal)
	{
		iBestYield = 0;

		for (iI = 0; iI < GC.getNumRouteInfos(); ++iI)
		{
			iBestYield = std::max(iBestYield, GC.getImprovementInfo(eImprovement).getRouteYieldChanges(iI, eYield));
		}

		iYield += iBestYield;
	}
	else
	{
		if (getRouteType() != NO_ROUTE)
		{
			iYield += GC.getImprovementInfo(eImprovement).getRouteYieldChanges(getRouteType(), eYield);
		}
	}

	if (bOptimal || ePlayer == NO_PLAYER)
	{
		for (iI = 0; iI < GC.getNumCivicInfos(); ++iI)
		{
			iYield += GC.getCivicInfo((CivicTypes) iI).getImprovementYieldChanges(eImprovement, eYield);
		}
	}
	else
	{
		iYield += GET_PLAYER(ePlayer).getImprovementYieldChange(eImprovement, eYield);
	}

	if (ePlayer != NO_PLAYER)
	{
		eBonus = getBonusType();
		if (eBonus != NO_BONUS)
		{
			iYield += GC.getImprovementInfo(eImprovement).getImprovementBonusYield(eBonus, eYield);
		}
	}

	if (GC.getImprovementInfo(eImprovement).getYieldIncrease(eYield) != 0)
	{
		TeamTypes eTeam = NO_TEAM;
		if (ePlayer != NO_PLAYER)
		{
			eTeam = GET_PLAYER(ePlayer).getTeam();
		}

		// TAC - AI Feature ignore for improvement previews fix - koma13 - START
		FeatureTypes eFeature = getFeatureType();
		bool bIgnoreFeature = false;

		if (eFeature != NO_FEATURE)
		{
			for (int iI = 0; iI < GC.getNumBuildInfos(); ++iI)
			{
				ImprovementTypes eLoopImprovement = ((ImprovementTypes)(GC.getBuildInfo((BuildTypes)iI).getImprovement()));

				if (eImprovement == eLoopImprovement)
				{
					if (GC.getBuildInfo((BuildTypes)iI).isFeatureRemove(eFeature))
					{
						bIgnoreFeature = true;
						break;
					}
				}
			}
		}
				
		//if (calculateNatureYield(eYield, eTeam, false) > 0)
		if (calculateNatureYield(eYield, eTeam, bIgnoreFeature) > 0)
		{
			iYield += GC.getImprovementInfo(eImprovement).getYieldIncrease(eYield);
		}
		// TAC - AI Feature ignore for improvement previews fix - koma13 - END
	}

	return iYield;
}

int CvPlot::calculatePotentialYield(YieldTypes eYield, const CvUnit* pUnit, bool bDisplay) const
{
	ImprovementTypes eImprovement;
	RouteTypes eRoute;
	PlayerTypes ePlayer;

	if (bDisplay)
	{
		ePlayer = getRevealedOwner(GC.getGameINLINE().getActiveTeam(), false);
		eImprovement = getRevealedImprovementType(GC.getGameINLINE().getActiveTeam(), false);
		eRoute = getRevealedRouteType(GC.getGameINLINE().getActiveTeam(), false);

		if (ePlayer == NO_PLAYER)
		{
			ePlayer = GC.getGameINLINE().getActivePlayer();
		}
	}
	else
	{
		ePlayer = getOwnerINLINE();
		eImprovement = getImprovementType();
		eRoute = getRouteType();
	}

	return calculatePotentialYield(eYield, ePlayer, eImprovement, false, eRoute, pUnit != NULL ? pUnit->getUnitType() : NO_UNIT, bDisplay);
}

int CvPlot::calculatePotentialYield(YieldTypes eYield, PlayerTypes ePlayer, ImprovementTypes eImprovement, bool bIgnoreFeature, RouteTypes eRoute, UnitTypes eUnit, bool bDisplay) const
{
	TeamTypes eTeam = ((ePlayer != NO_PLAYER) ? GET_PLAYER(ePlayer).getTeam() : NO_TEAM);

	if (getTerrainType() == NO_TERRAIN)
	{
		return 0;
	}

	//TAC Whaling, ray
	/*if (!isPotentialCityWork())
	{
		return 0;
	}*/

	int iYield = calculateNatureYield(eYield, eTeam, bIgnoreFeature);

	if (eImprovement != NO_IMPROVEMENT)
	{
		iYield += calculateImprovementYieldChange(eImprovement, eYield, ePlayer);
	}

	if (eRoute != NO_ROUTE)
	{
		if(iYield > 0)
		{
			iYield += GC.getRouteInfo(eRoute).getYieldChange(eYield);
		}
	}

	if (ePlayer != NO_PLAYER)
	{
		// R&R, ray, Landplot Yields - START
		if (!isWater())
		{
			if (!isImpassable())
			{
				CvCity* pWorkingCity = getWorkingCity();
				if (pWorkingCity != NULL)
				{
					if (!bDisplay || pWorkingCity->isRevealed(eTeam, false))
					{
						iYield += pWorkingCity->getLandPlotYield(eYield);
					}
				}
			}
		}
		// R&R, ray, Landplot Yields - END

		if (isWater())
		{
			if (!isImpassable())
			{
				iYield += GET_PLAYER(ePlayer).getSeaPlotYield(eYield);

				CvCity* pWorkingCity = getWorkingCity();
				if (pWorkingCity != NULL)
				{
					if (!bDisplay || pWorkingCity->isRevealed(eTeam, false))
					{
						iYield += pWorkingCity->getSeaPlotYield(eYield);
					}
				}
			}
		}

		if (isRiver())
		{
			if (!isImpassable())
			{
				CvCity* pWorkingCity = getWorkingCity();
				if (NULL != pWorkingCity)
				{
					if (!bDisplay || pWorkingCity->isRevealed(eTeam, false))
					{
						iYield += pWorkingCity->getRiverPlotYield(eYield);
					}
				}
			}
		}

		CvCity* pCity = getPlotCity();
		if (pCity != NULL)
		{
			if (!bDisplay || pCity->isRevealed(eTeam, false))
			{
				//city plot extra
				if (iYield > 0 || !GC.getYieldInfo(eYield).isCargo())
				{
					iYield += GC.getYieldInfo(eYield).getCityChange();
					iYield += GET_PLAYER(pCity->getOwnerINLINE()).getCityExtraYield(eYield);
				}

				if (eYield != YIELD_FOOD && GC.getYieldInfo(eYield).isCargo())
				{
					//cities get food and one other yield
					YieldTypes bestYield = pCity->getPreferredYieldAtCityPlot();
					int bestOutput = 0;
					
					// in display mode, only use the preferred yield for the owner's team
					// otherwise yield icons on the map will reveal the settings of the other players
					if (!bDisplay || getTeam() == GC.getGameINLINE().getActiveTeam() || GC.getGameINLINE().isDebugMode())
					{
						bestOutput = calculatePotentialCityYield(bestYield, pCity);
					}

					if (bestOutput == 0)
					{
						bestYield = NO_YIELD;
						for (int i = 0; i < NUM_YIELD_TYPES; i++)
						{
							int natureYield = calculatePotentialCityYield((YieldTypes)i, pCity);
							if (natureYield > bestOutput)
							{
								bestYield = (YieldTypes) i;
								bestOutput = natureYield;
							}
						}
					}

					if (eYield != bestYield)
					{
						iYield = 0;
					}
				}

				iYield = std::max(iYield, GC.getYieldInfo(eYield).getMinCity());
			}
		}
	}

	if (eUnit != NO_UNIT)
	{
		if (iYield > 0)
		{
			if (isValidYieldChanges(eUnit))
			{
				iYield += GC.getUnitInfo(eUnit).getYieldChange(eYield);

				if (getBonusType() != NO_BONUS)
				{
					if (GC.getBonusInfo(getBonusType()).getYieldChange(eYield) > 0)
					{
						iYield += GC.getUnitInfo(eUnit).getBonusYieldChange(eYield);
					}
				}
			}
		}
	}

	// R&R, Androrc, Livestock Breeding
	CvCity* pWorkingCity = getWorkingCity();
	if (pWorkingCity != NULL) 
	{
		//if (GC.getYieldInfo(eYield).isLivestock())
		if (GC.getYieldInfo(eYield).isLivestock() && (pWorkingCity->isHuman() || pWorkingCity->isNative())) // R&R, ray, Livestock Breeding, for AI
		{
			if (iYield > 0 && pWorkingCity->getYieldStored(eYield) > 0)
			{
				iYield = std::min(iYield, pWorkingCity->getYieldStored(eYield) * iYield / 100);
				if (iYield < 1)
				{
					iYield = 1;
				}
				if (pWorkingCity->isNative() && iYield < 4) // R&R, ray, Livestock Breeding, for AI
				{
					iYield = 4;
				}
			}
			else
			{
				iYield = 0;
			}
		}
	}
	// R&R, Androrc, Livestock Breeding, END

	iYield += GC.getGameINLINE().getPlotExtraYield(m_iX, m_iY, eYield);

	if (ePlayer != NO_PLAYER)
	{
		if (GET_PLAYER(ePlayer).getExtraYieldThreshold(eYield) > 0)
		{
			if (iYield >= GET_PLAYER(ePlayer).getExtraYieldThreshold(eYield))
			{
				iYield += GC.getDefineINT("EXTRA_YIELD");
			}
		}
	}

	int iModifier = 100;
	if (eUnit != NO_UNIT)
	{
		iModifier += GC.getUnitInfo(eUnit).getYieldModifier(eYield);
	}

	return std::max(0, (iYield * iModifier) / 100);
}
// R&R, ray , MYCP partially based on code of Aymerick - START
int CvPlot::calculatePotentialProfessionYieldAmount(ProfessionTypes eProfession, const CvUnit* pUnit, bool bDisplay) const
{
	int iYieldAmount = 0;

	if (NO_PROFESSION != eProfession)
	{
		CvProfessionInfo& kProfession = GC.getProfessionInfo(eProfession);
		if (kProfession.getYieldsProduced(0) != NO_YIELD)
		{
			if (isWater() == GC.getProfessionInfo(eProfession).isWater())
			{
				iYieldAmount = calculatePotentialYield((YieldTypes) kProfession.getYieldsProduced(0), pUnit, bDisplay);
			}
		}
	}

	return iYieldAmount;
}

int CvPlot::calculatePotentialProfessionYieldsAmount(YieldTypes eYield, ProfessionTypes eProfession, const CvUnit* pUnit, bool bDisplay) const
{
	int iYieldAmount = 0;

	if (NO_PROFESSION != eProfession)
	{
		CvProfessionInfo& kProfession = GC.getProfessionInfo(eProfession);
		if (eYield != NO_YIELD)
		{
			if (isWater() == GC.getProfessionInfo(eProfession).isWater())
			{
				iYieldAmount = calculatePotentialYield((YieldTypes) eYield, pUnit, bDisplay);
			}
		}
	}

	return iYieldAmount;
}
// R&R, ray , MYCP partially based on code of Aymerick - END

int CvPlot::calculatePotentialCityYield(YieldTypes eYield, const CvCity *pCity) const
{
	// callers don't verify if eYield is valid
	// NO_YIELD is frequently used
	if (eYield < 0 || eYield >= NUM_YIELD_TYPES)
	{
		return 0;
	}

	if (eYield == YIELD_FOOD || eYield == YIELD_LUMBER || eYield == YIELD_STONE || eYield == YIELD_HEMP)
	{
		return 0;
	}

	CvYieldInfo &kYieldInfo = GC.getYieldInfo(eYield);

	if (!kYieldInfo.isCargo() || kYieldInfo.isLivestock())
	{
		return 0;
	}

	int iYield = calculateNatureYield(eYield, pCity->getTeam(), false);

	if (iYield == 0)
	{
		return 0;
	}

	iYield += kYieldInfo.getCityChange();
	iYield += GET_PLAYER(pCity->getOwnerINLINE()).getCityExtraYield(eYield);

	return iYield;
}


int CvPlot::calculateYield(YieldTypes eYield, bool bDisplay) const
{
	const CvUnit* pUnit = NULL;
	//zero out all yields not produced by the working unit
	CvCity* pWorkingCity = getWorkingCity();
	// R&R, ray, small correction for balancing
	bool bSecondPlotYieldProduced = false;
	int iNewBalancedAmountForSecondPlotYield = 0;
	if (pWorkingCity != NULL)
	{
		pUnit = pWorkingCity->getUnitWorkingPlot(this);
		// R&R, ray , MYCP partially based on code of Aymerick - START
		bool bYieldProduced = false;
		if ((pUnit != NULL) && !bDisplay)
		{
			ProfessionTypes eProfession = pUnit->getProfession();
			if (NO_PROFESSION == eProfession)
			{
				return 0;
			}
			CvProfessionInfo& kProfession = GC.getProfessionInfo(eProfession);
			for (int i = 0; i < kProfession.getNumYieldsProduced(); i++)
			{
				YieldTypes eYieldProduced = (YieldTypes) kProfession.getYieldsProduced(i);
				if (eYieldProduced == eYield)
				{
					// R&R, ray, small correction for balancing
					if (i > 0 && kProfession.isWorkPlot())
					{
						bSecondPlotYieldProduced = true;
						iNewBalancedAmountForSecondPlotYield = calculatePotentialYield((YieldTypes) kProfession.getYieldsProduced(0), pUnit, bDisplay) / 2;
					}
					bYieldProduced = true;
				}
			}
			if (!bYieldProduced)
			{
				return 0;
			}
			if (kProfession.isWater() != isWater())
			{
				return 0;
			}
		}
		// R&R, ray , MYCP partially based on code of Aymerick - END
	}

	// R&R, ray, small correction for balancing
	if(bSecondPlotYieldProduced)
	{
		return iNewBalancedAmountForSecondPlotYield;
	}

	return calculatePotentialYield(eYield, pUnit, bDisplay);
}


bool CvPlot::hasYield() const
// CvPlot::hasYield cache - start - Nightinggale
{
	FAssert(m_bHasYield == hasYieldUncached());
	return m_bHasYield;
}

void CvPlot::setYieldCache()
{
	m_bHasYield = hasYieldUncached();
}

bool CvPlot::hasYieldUncached() const
// CvPlot::hasYield cache - end- Nightinggale
{
	for (int iI = 0; iI < NUM_YIELD_TYPES; ++iI)
	{
		if (calculatePotentialYield((YieldTypes)iI, NULL, false) > 0)
		{
			return true;
		}
	}

	return false;
}


void CvPlot::updateYield(bool bUpdateCity)
{
	CvCity* pWorkingCity;
	bool bChange;
	int iNewYield;
	int iOldYield;
	int iI;

	if (area() == NULL)
	{
		return;
	}

	bChange = false;

	for (iI = 0; iI < NUM_YIELD_TYPES; ++iI)
	{
		iNewYield = calculateYield((YieldTypes)iI, false);

		if (getYield((YieldTypes)iI) != iNewYield)
		{
			iOldYield = getYield((YieldTypes)iI);

			m_aiYield[iI] = iNewYield;
			FAssert(getYield((YieldTypes)iI) >= 0);

			if (bUpdateCity)
			{
				pWorkingCity = getWorkingCity();

				if (pWorkingCity != NULL)
				{
					if (isBeingWorked())
					{
						pWorkingCity->setYieldRateDirty();
					}

					pWorkingCity->AI_setAssignWorkDirty(true);
				}
			}

			bChange = true;
		}
	}

	if (bChange)
	{
		updateSymbols();
	}
}


int CvPlot::getCulture(PlayerTypes eIndex) const
{
	FAssertMsg(eIndex >= 0, "iIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "iIndex is expected to be within maximum bounds (invalid Index)");

	if (NULL == m_aiCulture)
	{
		return 0;
	}

	return m_aiCulture[eIndex];
}


int CvPlot::countTotalCulture() const
{
	int iTotalCulture;
	int iI;

	iTotalCulture = 0;

	for (iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		if (GET_PLAYER((PlayerTypes)iI).isAlive())
		{
			iTotalCulture += getCulture((PlayerTypes)iI);
		}
	}

	return iTotalCulture;
}


TeamTypes CvPlot::findHighestCultureTeam() const
{
	PlayerTypes eBestPlayer = findHighestCulturePlayer();

	if (NO_PLAYER == eBestPlayer)
	{
		return NO_TEAM;
	}

	return GET_PLAYER(eBestPlayer).getTeam();
}


PlayerTypes CvPlot::findHighestCulturePlayer() const
{
	PlayerTypes eBestPlayer = NO_PLAYER;
	int iBestValue = 0;

	for (int iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		if (GET_PLAYER((PlayerTypes)iI).isAlive())
		{
			int iValue = getCulture((PlayerTypes)iI);

			if (iValue > iBestValue)
			{
				iBestValue = iValue;
				eBestPlayer = (PlayerTypes)iI;
			}
		}
	}

	return eBestPlayer;
}


int CvPlot::calculateCulturePercent(PlayerTypes eIndex) const
{
	int iTotalCulture;

	iTotalCulture = countTotalCulture();

	if (iTotalCulture > 0)
	{
		return ((getCulture(eIndex) * 100) / iTotalCulture);
	}

	return 0;
}


int CvPlot::calculateTeamCulturePercent(TeamTypes eIndex) const
{
	int iTeamCulturePercent;
	int iI;

	iTeamCulturePercent = 0;

	for (iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		if (GET_PLAYER((PlayerTypes)iI).isAlive())
		{
			if (GET_PLAYER((PlayerTypes)iI).getTeam() == eIndex)
			{
				iTeamCulturePercent += calculateCulturePercent((PlayerTypes)iI);
			}
		}
	}

	return iTeamCulturePercent;
}


void CvPlot::setCulture(PlayerTypes eIndex, int iNewValue, bool bUpdate)
{
	PROFILE_FUNC();

	CvCity* pCity;

	FAssertMsg(eIndex >= 0, "iIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "iIndex is expected to be within maximum bounds (invalid Index)");

	if (getCulture(eIndex) != iNewValue)
	{
		if(NULL == m_aiCulture)
		{
			m_aiCulture = new int[MAX_PLAYERS];
			for (int iI = 0; iI < MAX_PLAYERS; ++iI)
			{
				m_aiCulture[iI] = 0;
			}
		}

		m_aiCulture[eIndex] = iNewValue;
		FAssert(getCulture(eIndex) >= 0);

		if (bUpdate)
		{
			updateCulture(true);
		}

		pCity = getPlotCity();

		if (pCity != NULL)
		{
			pCity->AI_setAssignWorkDirty(true);
		}
	}
}


void CvPlot::changeCulture(PlayerTypes eIndex, int iChange, bool bUpdate)
{
	if (0 != iChange)
	{
		setCulture(eIndex, (getCulture(eIndex) + iChange), bUpdate);
	}
}

int CvPlot::getBuyCultureAmount(PlayerTypes ePlayer) const
{
	int iCulture = GC.getDefineINT("BUY_PLOT_MIN_CULTURE") * GC.getGameSpeedInfo(GC.getGameINLINE().getGameSpeedType()).getGrowthPercent() / 100;

	if (getOwnerINLINE() != NO_PLAYER)
	{
		iCulture = std::max(iCulture, getCulture(getOwnerINLINE()));
	}

	return iCulture;
}


int CvPlot::getBuyPrice(PlayerTypes ePlayer) const
{
	int iBuyPrice = getBuyCultureAmount(ePlayer) * GC.getDefineINT("BUY_PLOT_BASE_CULTURE_COST");

	int iModifier = 100;
	if (getOwnerINLINE() != NO_PLAYER)
	{
		if (!GET_TEAM(GET_PLAYER(ePlayer).getTeam()).isAtWar(getTeam()))
		{
			iModifier += GC.getDefineINT("BUY_PLOT_OWNED_COST_MODIFIER");
		}

		for (int iTrait = 0; iTrait < GC.getNumTraitInfos(); ++iTrait)
		{
			if (GET_PLAYER(getOwnerINLINE()).hasTrait((TraitTypes) iTrait))
			{
				iModifier *= 100 + GC.getTraitInfo((TraitTypes) iTrait).getLandPriceDiscount();
				iModifier /= 100;
			}
		}
	}

	for (int iTrait = 0; iTrait < GC.getNumTraitInfos(); ++iTrait)
	{
		if (GET_PLAYER(ePlayer).hasTrait((TraitTypes) iTrait))
		{
			iModifier *= 100 + GC.getTraitInfo((TraitTypes) iTrait).getLandPriceDiscount();
			iModifier /= 100;
		}
	}

	iBuyPrice *= std::max(0, iModifier);
	iBuyPrice /= 100;

	return iBuyPrice;
}


int CvPlot::getFoundValue(PlayerTypes eIndex)
{
	FAssertMsg(eIndex >= 0, "eIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "eIndex is expected to be within maximum bounds (invalid Index)");

	if (NULL == m_aiFoundValue)
	{
		return 0;
	}

	if (m_aiFoundValue[eIndex] == -1)
	{
		long lResult=-1;
		if(GC.getUSE_GET_CITY_FOUND_VALUE_CALLBACK())
		{
			CyArgsList argsList;
			argsList.add((int)eIndex);
			argsList.add(getX());
			argsList.add(getY());
			gDLL->getPythonIFace()->callFunction(PYGameModule, "getCityFoundValue", argsList.makeFunctionArgs(), &lResult);
		}

		if (lResult == -1)
		{
			m_aiFoundValue[eIndex] = GET_PLAYER(eIndex).AI_foundValue(getX_INLINE(), getY_INLINE(), -1, true);
		}

		if (m_aiFoundValue[eIndex] > area()->getBestFoundValue(eIndex))
		{
			area()->setBestFoundValue(eIndex, m_aiFoundValue[eIndex]);
		}
	}

	return m_aiFoundValue[eIndex];
}


bool CvPlot::isBestAdjacentFound(PlayerTypes eIndex)
{
	CvPlot* pAdjacentPlot;
	int iI;

	int iPlotValue = GET_PLAYER(eIndex).AI_foundValue(getX_INLINE(), getY_INLINE());

	if (iPlotValue == 0)
	{
		return false;
	}

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if ((pAdjacentPlot != NULL) && pAdjacentPlot->isRevealed(GET_PLAYER(eIndex).getTeam(), false))
		{
			//if (pAdjacentPlot->getFoundValue(eIndex) >= getFoundValue(eIndex))
			if (GET_PLAYER(eIndex).AI_foundValue(pAdjacentPlot->getX_INLINE(), pAdjacentPlot->getY_INLINE()) > iPlotValue)
			{
				return false;
			}
		}
	}

	return true;
}


void CvPlot::setFoundValue(PlayerTypes eIndex, int iNewValue)
{
	FAssertMsg(eIndex >= 0, "eIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "eIndex is expected to be within maximum bounds (invalid Index)");
	FAssert(iNewValue >= -1);

	if (NULL == m_aiFoundValue && 0 != iNewValue)
	{
		m_aiFoundValue = new int[MAX_PLAYERS];
		for (int iI = 0; iI < MAX_PLAYERS; ++iI)
		{
			m_aiFoundValue[iI] = 0;
		}
	}

	if (NULL != m_aiFoundValue)
	{
		m_aiFoundValue[eIndex] = iNewValue;
	}
}


int CvPlot::getPlayerCityRadiusCount(PlayerTypes eIndex) const
{
	FAssertMsg(eIndex >= 0, "eIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "eIndex is expected to be within maximum bounds (invalid Index)");

	if (NULL == m_aiPlayerCityRadiusCount)
	{
		return 0;
	}

	return m_aiPlayerCityRadiusCount[eIndex];
}


bool CvPlot::isPlayerCityRadius(PlayerTypes eIndex) const
{
	return (getPlayerCityRadiusCount(eIndex) > 0);
}


void CvPlot::changePlayerCityRadiusCount(PlayerTypes eIndex, int iChange)
{
	FAssertMsg(eIndex >= 0, "eIndex is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < MAX_PLAYERS, "eIndex is expected to be within maximum bounds (invalid Index)");

	if (0 != iChange)
	{
		if (NULL == m_aiPlayerCityRadiusCount)
		{
			m_aiPlayerCityRadiusCount = new char[MAX_PLAYERS];
			for (int iI = 0; iI < MAX_PLAYERS; ++iI)
			{
				m_aiPlayerCityRadiusCount[iI] = 0;
			}
		}
		
		m_aiPlayerCityRadiusCount[eIndex] += iChange;
		//R&R mod, vetiarvind, bug fix for city radius going below zero. - start
		if(m_aiPlayerCityRadiusCount[eIndex] < 0) 		
			m_aiPlayerCityRadiusCount[eIndex] = 0;		
		//R&R mod, vetiarvind, bug fix for city radius going below zero. - end
		FAssert(getPlayerCityRadiusCount(eIndex) >= 0);
	}
}


int CvPlot::getVisibilityCount(TeamTypes eTeam) const
{
	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	if (NULL == m_aiVisibilityCount)
	{
		return 0;
	}

	return m_aiVisibilityCount[eTeam];
}


void CvPlot::changeVisibilityCount(TeamTypes eTeam, int iChange, InvisibleTypes eSeeInvisible)
{
	CvCity* pCity;
	CvPlot* pAdjacentPlot;
	bool bOldVisible;
	int iI;

	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	if (iChange != 0)
	{
		if (NULL == m_aiVisibilityCount)
		{
			m_aiVisibilityCount = new short[MAX_TEAMS];
			for (int iI = 0; iI < MAX_TEAMS; ++iI)
			{
				m_aiVisibilityCount[iI] = 0;
			}
		}

		bOldVisible = isVisible(eTeam, false);

		m_aiVisibilityCount[eTeam] += iChange;
		//R&R mod, vetiarvind, bug fix for visibilty going below zero. - start
		if(m_aiVisibilityCount[eTeam] < 0)
			m_aiVisibilityCount[eTeam] = 0;
		//R&R mod, vetiarvind, bug fix for visibilty going below zero. - end
		FAssert(getVisibilityCount(eTeam) >= 0);

		if (eSeeInvisible != NO_INVISIBLE)
		{
			changeInvisibleVisibilityCount(eTeam, eSeeInvisible, iChange);
		}

		if (bOldVisible != isVisible(eTeam, false))
		{
			if (isVisible(eTeam, false))
			{
				setRevealed(eTeam, true, false, NO_TEAM);

				for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
				{
					pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

					if (pAdjacentPlot != NULL)
					{
						pAdjacentPlot->updateRevealedOwner(eTeam);
					}
				}
			}

			pCity = getPlotCity();

			if (pCity != NULL)
			{
				pCity->setBillboardDirty(true);
			}

			if (eTeam == GC.getGameINLINE().getActiveTeam())
			{
				updateFog();
				updateMinimapColor();
				updateCenterUnit();
			}
		}
	}
}


PlayerTypes CvPlot::getRevealedOwner(TeamTypes eTeam, bool bDebug) const
{
	if (bDebug && GC.getGameINLINE().isDebugMode())
	{
		return getOwnerINLINE();
	}
	else
	{
		FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
		FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

		if (NULL == m_aiRevealedOwner)
		{
			return NO_PLAYER;
		}

		return (PlayerTypes)m_aiRevealedOwner[eTeam];
	}
}


TeamTypes CvPlot::getRevealedTeam(TeamTypes eTeam, bool bDebug) const
{
	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	PlayerTypes eRevealedOwner = getRevealedOwner(eTeam, bDebug);

	if (eRevealedOwner != NO_PLAYER)
	{
		return GET_PLAYER(eRevealedOwner).getTeam();
	}
	else
	{
		return NO_TEAM;
	}
}


void CvPlot::setRevealedOwner(TeamTypes eTeam, PlayerTypes eNewValue)
{
	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	if (getRevealedOwner(eTeam, false) != eNewValue)
	{
		if (NULL == m_aiRevealedOwner)
		{
			m_aiRevealedOwner = new char[MAX_TEAMS];
			for (int iI = 0; iI < MAX_TEAMS; ++iI)
			{
				m_aiRevealedOwner[iI] = -1;
			}
		}

		m_aiRevealedOwner[eTeam] = eNewValue;

		if (eTeam == GC.getGameINLINE().getActiveTeam())
		{
			updateMinimapColor();

			if (GC.IsGraphicsInitialized())
			{
				gDLL->getInterfaceIFace()->setDirty(GlobeLayer_DIRTY_BIT, true);

				gDLL->getEngineIFace()->SetDirty(CultureBorders_DIRTY_BIT, true);
			}
		}
	}

	FAssert((NULL == m_aiRevealedOwner) || (m_aiRevealedOwner[eTeam] == eNewValue));
}


void CvPlot::updateRevealedOwner(TeamTypes eTeam)
{
	CvPlot* pAdjacentPlot;
	bool bRevealed;
	int iI;

	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	bRevealed = false;

	if (!bRevealed)
	{
		if (isVisible(eTeam, false))
		{
			bRevealed = true;
		}
	}

	if (!bRevealed)
	{
		for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

			if (pAdjacentPlot != NULL)
			{
				if (pAdjacentPlot->isVisible(eTeam, false))
				{
					bRevealed = true;
					break;
				}
			}
		}
	}

	if (bRevealed)
	{
		setRevealedOwner(eTeam, getOwnerINLINE());
	}
}


bool CvPlot::isRiverCrossing(DirectionTypes eIndex) const
{
	FAssertMsg(eIndex < NUM_DIRECTION_TYPES, "eTeam is expected to be within maximum bounds (invalid Index)");

	if (eIndex == NO_DIRECTION)
	{
		return false;
	}

	if (NULL == m_abRiverCrossing)
	{
		return false;
	}

	return m_abRiverCrossing[eIndex];
}


void CvPlot::updateRiverCrossing(DirectionTypes eIndex)
{
	CvPlot* pNorthEastPlot;
	CvPlot* pSouthEastPlot;
	CvPlot* pSouthWestPlot;
	CvPlot* pNorthWestPlot;
	CvPlot* pCornerPlot;
	CvPlot* pPlot;
	bool bValid;

	FAssertMsg(eIndex >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eIndex < NUM_DIRECTION_TYPES, "eTeam is expected to be within maximum bounds (invalid Index)");

	pCornerPlot = NULL;
	bValid = false;
	pPlot = plotDirection(getX_INLINE(), getY_INLINE(), eIndex);

	if ((NULL == pPlot || !pPlot->isWater()) && !isWater())
	{
		switch (eIndex)
		{
		case DIRECTION_NORTH:
			if (pPlot != NULL)
			{
				bValid = pPlot->isNOfRiver();
			}
			break;

		case DIRECTION_NORTHEAST:
			pCornerPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_NORTH);
			break;

		case DIRECTION_EAST:
			bValid = isWOfRiver();
			break;

		case DIRECTION_SOUTHEAST:
			pCornerPlot = this;
			break;

		case DIRECTION_SOUTH:
			bValid = isNOfRiver();
			break;

		case DIRECTION_SOUTHWEST:
			pCornerPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_WEST);
			break;

		case DIRECTION_WEST:
			if (pPlot != NULL)
			{
				bValid = pPlot->isWOfRiver();
			}
			break;

		case DIRECTION_NORTHWEST:
			pCornerPlot = plotDirection(getX_INLINE(), getY_INLINE(), DIRECTION_NORTHWEST);
			break;

		default:
			FAssert(false);
			break;
		}

		if (pCornerPlot != NULL)
		{
			pNorthEastPlot = plotDirection(pCornerPlot->getX_INLINE(), pCornerPlot->getY_INLINE(), DIRECTION_EAST);
			pSouthEastPlot = plotDirection(pCornerPlot->getX_INLINE(), pCornerPlot->getY_INLINE(), DIRECTION_SOUTHEAST);
			pSouthWestPlot = plotDirection(pCornerPlot->getX_INLINE(), pCornerPlot->getY_INLINE(), DIRECTION_SOUTH);
			pNorthWestPlot = pCornerPlot;

			if (pSouthWestPlot && pNorthWestPlot && pSouthEastPlot && pNorthEastPlot)
			{
				if (pSouthWestPlot->isWOfRiver() && pNorthWestPlot->isWOfRiver())
				{
					bValid = true;
				}
				else if (pNorthEastPlot->isNOfRiver() && pNorthWestPlot->isNOfRiver())
				{
					bValid = true;
				}
				else if ((eIndex == DIRECTION_NORTHEAST) || (eIndex == DIRECTION_SOUTHWEST))
				{
					if (pNorthEastPlot->isNOfRiver() && (pNorthWestPlot->isWOfRiver() || pNorthWestPlot->isWater()))
					{
						bValid = true;
					}
					else if ((pNorthEastPlot->isNOfRiver() || pSouthEastPlot->isWater()) && pNorthWestPlot->isWOfRiver())
					{
						bValid = true;
					}
					else if (pSouthWestPlot->isWOfRiver() && (pNorthWestPlot->isNOfRiver() || pNorthWestPlot->isWater()))
					{
						bValid = true;
					}
					else if ((pSouthWestPlot->isWOfRiver() || pSouthEastPlot->isWater()) && pNorthWestPlot->isNOfRiver())
					{
						bValid = true;
					}
				}
				else
				{
					FAssert((eIndex == DIRECTION_SOUTHEAST) || (eIndex == DIRECTION_NORTHWEST));

					if (pNorthWestPlot->isNOfRiver() && (pNorthWestPlot->isWOfRiver() || pNorthEastPlot->isWater()))
					{
						bValid = true;
					}
					else if ((pNorthWestPlot->isNOfRiver() || pSouthWestPlot->isWater()) && pNorthWestPlot->isWOfRiver())
					{
						bValid = true;
					}
					else if (pNorthEastPlot->isNOfRiver() && (pSouthWestPlot->isWOfRiver() || pSouthWestPlot->isWater()))
					{
						bValid = true;
					}
					else if ((pNorthEastPlot->isNOfRiver() || pNorthEastPlot->isWater()) && pSouthWestPlot->isWOfRiver())
					{
						bValid = true;
					}
				}
			}

		}
	}

	if (isRiverCrossing(eIndex) != bValid)
	{
		if (NULL == m_abRiverCrossing)
		{
			m_abRiverCrossing = new bool[NUM_DIRECTION_TYPES];
			for (int iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
			{
				m_abRiverCrossing[iI] = false;
			}
		}

		m_abRiverCrossing[eIndex] = bValid;

		changeRiverCrossingCount((isRiverCrossing(eIndex)) ? 1 : -1);
	}
}


void CvPlot::updateRiverCrossing()
{
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		updateRiverCrossing((DirectionTypes)iI);
	}
}


bool CvPlot::isRevealed(TeamTypes eTeam, bool bDebug) const
{
	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	if (bDebug && GC.getGameINLINE().isDebugMode())
	{
		return true;
	}

	if (NULL == m_abRevealed)
	{
		return false;
	}

	return m_abRevealed[eTeam];
}


void CvPlot::setRevealed(TeamTypes eTeam, bool bNewValue, bool bTerrainOnly, TeamTypes eFromTeam)
{
	CvCity* pCity;

	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	pCity = getPlotCity();

	if (isRevealed(eTeam, false) != bNewValue)
	{
		if (NULL == m_abRevealed)
		{
			m_abRevealed = new bool[MAX_TEAMS];
			for (int iI = 0; iI < MAX_TEAMS; ++iI)
			{
				m_abRevealed[iI] = false;
			}
		}

		m_abRevealed[eTeam] = bNewValue;

		if (area())
		{
			area()->changeNumRevealedTiles(eTeam, ((isRevealed(eTeam, false)) ? 1 : -1));
		}

		if (eTeam == GC.getGameINLINE().getActiveTeam())
		{
			updateSymbols();
			updateFog();
			updateVisibility();

			gDLL->getInterfaceIFace()->setDirty(MinimapSection_DIRTY_BIT, true);
			gDLL->getInterfaceIFace()->setDirty(GlobeLayer_DIRTY_BIT, true);
			gDLL->getInterfaceIFace()->setDirty(ColoredPlots_DIRTY_BIT, true);
		}

		if (isRevealed(eTeam, false))
		{
			for (int i = 0; i < GC.getNumFatherPointInfos(); ++i)
			{
				FatherPointTypes ePointType = (FatherPointTypes) i;
				int gameSpeedMod =  GC.getGameSpeedInfo(GC.getGameINLINE().getGameSpeedType()).getGrowthPercent();
				int tilePoints = 3; //will change
				if (isWater())
				{
					tilePoints =  GC.getFatherPointInfo(ePointType).getWaterTilePoints();
				}
				else
				{
					tilePoints = GC.getFatherPointInfo(ePointType).getLandTilePoints();
				}
				tilePoints *= gameSpeedMod * GET_TEAM(eTeam).getBestFatherPointMultiplier()  /10000;
				GET_TEAM(eTeam).changeFatherPoints(ePointType, tilePoints);
			}

			// ONEVENT - PlotRevealed
			gDLL->getEventReporterIFace()->plotRevealed(this, eTeam);
		}
	}

	if (!bTerrainOnly)
	{
		if (isRevealed(eTeam, false))
		{
			if (eFromTeam == NO_TEAM)
			{
				setRevealedOwner(eTeam, getOwnerINLINE());
				setRevealedImprovementType(eTeam, getImprovementType());
				setRevealedRouteType(eTeam, getRouteType());

				if (pCity != NULL)
				{
					pCity->setRevealed(eTeam, true);
				}
			}
			else
			{
				if (getRevealedOwner(eFromTeam, false) == getOwnerINLINE())
				{
					setRevealedOwner(eTeam, getRevealedOwner(eFromTeam, false));
				}

				if (getRevealedImprovementType(eFromTeam, false) == getImprovementType())
				{
					setRevealedImprovementType(eTeam, getRevealedImprovementType(eFromTeam, false));
				}

				if (getRevealedRouteType(eFromTeam, false) == getRouteType())
				{
					setRevealedRouteType(eTeam, getRevealedRouteType(eFromTeam, false));
				}

				if (pCity != NULL)
				{
					if (pCity->isRevealed(eFromTeam, false))
					{
						pCity->setRevealed(eTeam, true);
					}
				}
			}
		}
		else
		{
			setRevealedOwner(eTeam, NO_PLAYER);
			setRevealedImprovementType(eTeam, NO_IMPROVEMENT);
			setRevealedRouteType(eTeam, NO_ROUTE);

			if (pCity != NULL)
			{
				pCity->setRevealed(eTeam, false);
			}
		}
	}
}

bool CvPlot::isAdjacentRevealed(TeamTypes eTeam) const
{
	CvPlot* pAdjacentPlot;
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (pAdjacentPlot->isRevealed(eTeam, false))
			{
				return true;
			}
		}
	}

	return false;
}

bool CvPlot::isAdjacentNonrevealed(TeamTypes eTeam) const
{
	CvPlot* pAdjacentPlot;
	int iI;

	for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
	{
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

		if (pAdjacentPlot != NULL)
		{
			if (!pAdjacentPlot->isRevealed(eTeam, false))
			{
				return true;
			}
		}
	}

	return false;
}


ImprovementTypes CvPlot::getRevealedImprovementType(TeamTypes eTeam, bool bDebug) const
{
	if (bDebug && GC.getGameINLINE().isDebugMode())
	{
		return getImprovementType();
	}
	else
	{
		FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
		FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

		if (NULL == m_aeRevealedImprovementType)
		{
			return NO_IMPROVEMENT;
		}

		return (ImprovementTypes)m_aeRevealedImprovementType[eTeam];
	}
}


void CvPlot::setRevealedImprovementType(TeamTypes eTeam, ImprovementTypes eNewValue)
{
	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	if (getRevealedImprovementType(eTeam, false) != eNewValue)
	{
		if (NULL == m_aeRevealedImprovementType)
		{
			m_aeRevealedImprovementType = new short[MAX_TEAMS];
			for (int iI = 0; iI < MAX_TEAMS; ++iI)
			{
				m_aeRevealedImprovementType[iI] = NO_IMPROVEMENT;
			}
		}

		m_aeRevealedImprovementType[eTeam] = eNewValue;

		if (eTeam == GC.getGameINLINE().getActiveTeam())
		{
			updateSymbols();
			setLayoutDirty(true);
			//gDLL->getEngineIFace()->SetDirty(GlobeTexture_DIRTY_BIT, true);
		}
	}
}


RouteTypes CvPlot::getRevealedRouteType(TeamTypes eTeam, bool bDebug) const
{
	if (bDebug && GC.getGameINLINE().isDebugMode())
	{
		return getRouteType();
	}
	else
	{
		FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
		FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

		if (NULL == m_aeRevealedRouteType)
		{
			return NO_ROUTE;
		}

		return (RouteTypes)m_aeRevealedRouteType[eTeam];
	}
}


void CvPlot::setRevealedRouteType(TeamTypes eTeam, RouteTypes eNewValue)
{
	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");

	if (getRevealedRouteType(eTeam, false) != eNewValue)
	{
		if (NULL == m_aeRevealedRouteType)
		{
			m_aeRevealedRouteType = new short[MAX_TEAMS];
			for (int iI = 0; iI < MAX_TEAMS; ++iI)
			{
				m_aeRevealedRouteType[iI] = NO_ROUTE;
			}
		}

		m_aeRevealedRouteType[eTeam] = eNewValue;

		if (eTeam == GC.getGameINLINE().getActiveTeam())
		{
			updateSymbols();
			updateRouteSymbol(true, true);
		}
	}
}


int CvPlot::getBuildProgress(BuildTypes eBuild) const
{
	if (NULL == m_paiBuildProgress)
	{
		return 0;
	}

	return m_paiBuildProgress[eBuild];
}


// Returns true if build finished...
bool CvPlot::changeBuildProgress(BuildTypes eBuild, int iChange, TeamTypes eTeam)
{
	CvWString szBuffer;
	bool bFinished = false;

	if (iChange != 0)
	{
		if (NULL == m_paiBuildProgress)
		{
			m_paiBuildProgress = new short[GC.getNumBuildInfos()];
			for (int iI = 0; iI < GC.getNumBuildInfos(); ++iI)
			{
				m_paiBuildProgress[iI] = 0;
			}
		}

		m_paiBuildProgress[eBuild] += iChange;
		FAssert(getBuildProgress(eBuild) >= 0);

		if (getBuildProgress(eBuild) >= getBuildTime(eBuild))
		{
			m_paiBuildProgress[eBuild] = 0;

			if (GC.getBuildInfo(eBuild).getImprovement() != NO_IMPROVEMENT)
			{
				setImprovementType((ImprovementTypes)GC.getBuildInfo(eBuild).getImprovement());
			}

			if (GC.getBuildInfo(eBuild).getRoute() != NO_ROUTE)
			{
				setRouteType((RouteTypes)GC.getBuildInfo(eBuild).getRoute());
			}

			if (getFeatureType() != NO_FEATURE)
			{
				if (GC.getBuildInfo(eBuild).isFeatureRemove(getFeatureType()))
				{
					FAssertMsg(eTeam != NO_TEAM, "eTeam should be valid");

					CvCity* pCity = NULL;
					for (int iYield = 0; iYield < NUM_YIELD_TYPES; ++iYield)
					{
						YieldTypes eYield = (YieldTypes) iYield;
						if (GC.getYieldInfo(eYield).isCargo())
						{
							int iYieldProduction = getFeatureYield(eBuild, eYield, eTeam, &pCity);
							if (iYieldProduction > 0)
							{
								pCity->changeYieldStored(eYield, iYieldProduction);

								szBuffer = gDLL->getText("TXT_KEY_MISC_CLEARING_FEATURE_BONUS", GC.getFeatureInfo(getFeatureType()).getTextKeyWide(), iYieldProduction, pCity->getNameKey(), GC.getYieldInfo(eYield).getChar());
								gDLL->getInterfaceIFace()->addMessage(pCity->getOwnerINLINE(), false, GC.getEVENT_MESSAGE_TIME(), szBuffer,  ARTFILEMGR.getInterfaceArtInfo("WORLDBUILDER_CITY_EDIT")->getPath(), MESSAGE_TYPE_INFO, GC.getFeatureInfo(getFeatureType()).getButton(), (ColorTypes)GC.getInfoTypeForString("COLOR_WHITE"), getX_INLINE(), getY_INLINE(), true, true);
							}
						}
					}

					// Python Event
					gDLL->getEventReporterIFace()->plotFeatureRemoved(this, getFeatureType(), pCity);

					setFeatureType(NO_FEATURE);
				}
			}

			// R&R, ray, Terraforming Features - START
			if (GC.getBuildInfo(eBuild).getResultTerrain() != NO_TERRAIN)
			{
				setTerrainType((TerrainTypes)GC.getBuildInfo(eBuild).getResultTerrain());
			}

			if (GC.getBuildInfo(eBuild).getResultFeature() != NO_FEATURE)
			{
				setImprovementType(NO_IMPROVEMENT);
				setFeatureType((FeatureTypes)GC.getBuildInfo(eBuild).getResultFeature());
			}
			// R&R, ray, Terraforming Features - END

			bFinished = true;
		}
	}

	return bFinished;
}


void CvPlot::updateFeatureSymbolVisibility()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	if (m_pFeatureSymbol != NULL)
	{
		bool bVisible = isRevealed(GC.getGameINLINE().getActiveTeam(), true);
		if(getFeatureType() != NO_FEATURE)
		{
			if(GC.getFeatureInfo(getFeatureType()).isVisibleAlways())
				bVisible = true;
		}

		bool wasVisible = !gDLL->getFeatureIFace()->IsHidden(m_pFeatureSymbol);
		if(wasVisible != bVisible)
		{
			gDLL->getFeatureIFace()->Hide(m_pFeatureSymbol, !bVisible);
			gDLL->getEngineIFace()->MarkPlotTextureAsDirty(m_iX, m_iY);
		}
	}
}


void CvPlot::updateFeatureSymbol(bool bForce, bool bBuildTileArt)
{
	PROFILE_FUNC();

	FeatureTypes eFeature;

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	eFeature = getFeatureType();

	if(bBuildTileArt)
	{
		gDLL->getEngineIFace()->RebuildTileArt(m_iX,m_iY);
	}

	if ((eFeature == NO_FEATURE) || (GC.getFeatureInfo(eFeature).getArtInfo()->getTileArtType() != TILE_ART_TYPE_NONE))
	{
		gDLL->getFeatureIFace()->destroy(m_pFeatureSymbol);
		return;
	}

	if (bForce || (m_pFeatureSymbol == NULL) || (gDLL->getFeatureIFace()->getFeature(m_pFeatureSymbol) != eFeature))
	{
		gDLL->getFeatureIFace()->destroy(m_pFeatureSymbol);
		m_pFeatureSymbol = gDLL->getFeatureIFace()->createFeature();

		FAssertMsg(m_pFeatureSymbol != NULL, "m_pFeatureSymbol is not expected to be equal with NULL");

		gDLL->getFeatureIFace()->init(m_pFeatureSymbol, 0, 0, eFeature, this);

		updateFeatureSymbolVisibility();
	}
	else
	{
		gDLL->getEntityIFace()->updatePosition((CvEntity*)m_pFeatureSymbol); //update position and contours
	}
}


CvRoute* CvPlot::getRouteSymbol() const
{
	return m_pRouteSymbol;
}


// XXX route symbols don't really exist anymore...
void CvPlot::updateRouteSymbol(bool bForce, bool bAdjacent)
{
	PROFILE_FUNC();

	CvPlot* pAdjacentPlot;
	RouteTypes eRoute;
	int iI;

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	if (bAdjacent)
	{
		for (iI = 0; iI < NUM_DIRECTION_TYPES; ++iI)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));

			if (pAdjacentPlot != NULL)
			{
				pAdjacentPlot->updateRouteSymbol(bForce, false);
				//pAdjacentPlot->setLayoutDirty(true);
			}
		}
	}

	eRoute = getRevealedRouteType(GC.getGameINLINE().getActiveTeam(), true);

	if (eRoute == NO_ROUTE)
	{
		gDLL->getRouteIFace()->destroy(m_pRouteSymbol);
		return;
	}

	if (bForce || (m_pRouteSymbol == NULL) || (gDLL->getRouteIFace()->getRoute(m_pRouteSymbol) != eRoute))
	{
		gDLL->getRouteIFace()->destroy(m_pRouteSymbol);
		m_pRouteSymbol = gDLL->getRouteIFace()->createRoute();
		FAssertMsg(m_pRouteSymbol != NULL, "m_pRouteSymbol is not expected to be equal with NULL");

		gDLL->getRouteIFace()->init(m_pRouteSymbol, 0, 0, eRoute, this);
		setLayoutDirty(true);
	}
	else
	{
		gDLL->getEntityIFace()->updatePosition((CvEntity *)m_pRouteSymbol); //update position and contours
	}
}


CvRiver* CvPlot::getRiverSymbol() const
{
	return m_pRiverSymbol;
}


CvFeature* CvPlot::getFeatureSymbol() const
{
	return m_pFeatureSymbol;
}


void CvPlot::updateRiverSymbol(bool bForce, bool bAdjacent)
{
	PROFILE_FUNC();

	CvPlot* pAdjacentPlot;

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	if (bAdjacent)
	{
		for(int i=0;i<NUM_DIRECTION_TYPES;i++)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)i));
			if (pAdjacentPlot != NULL)
			{
				pAdjacentPlot->updateRiverSymbol(bForce, false);
				//pAdjacentPlot->setLayoutDirty(true);
			}
		}
	}

	if (!isRiverMask())
	{
		gDLL->getRiverIFace()->destroy(m_pRiverSymbol);
		return;
	}

	if (bForce || (m_pRiverSymbol == NULL))
	{
		//create river
		gDLL->getRiverIFace()->destroy(m_pRiverSymbol);
		m_pRiverSymbol = gDLL->getRiverIFace()->createRiver();
		FAssertMsg(m_pRiverSymbol != NULL, "m_pRiverSymbol is not expected to be equal with NULL");
		gDLL->getRiverIFace()->init(m_pRiverSymbol, 0, 0, 0, this);

		//cut out canyons
		gDLL->getEngineIFace()->RebuildRiverPlotTile(getX_INLINE(), getY_INLINE(), true, false);

		//recontour adjacent rivers
		for(int i=0;i<NUM_DIRECTION_TYPES;i++)
		{
			pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)i));
			if((pAdjacentPlot != NULL) && (pAdjacentPlot->m_pRiverSymbol != NULL))
			{
				gDLL->getEntityIFace()->updatePosition((CvEntity *)pAdjacentPlot->m_pRiverSymbol); //update position and contours
			}
		}

		// update the symbol
		setLayoutDirty(true);
	}

	//recontour rivers
	gDLL->getEntityIFace()->updatePosition((CvEntity *)m_pRiverSymbol); //update position and contours
}


CvFlagEntity* CvPlot::getFlagSymbol() const
{
	return m_pFlagSymbol;
}

CvFlagEntity* CvPlot::getFlagSymbolOffset() const
{
	return m_pFlagSymbolOffset;
}

void CvPlot::updateFlagSymbol()
{
	PROFILE_FUNC();

	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	PlayerTypes ePlayer = NO_PLAYER;
	PlayerTypes ePlayerOffset = NO_PLAYER;

	CvUnit* pCenterUnit = getCenterUnit();

	//get the plot's unit's flag
	if (pCenterUnit != NULL)
	{
		ePlayer = pCenterUnit->getVisualOwner();
	}

	//get moving unit's flag
	if (gDLL->getInterfaceIFace()->getSingleMoveGotoPlot() == this)
	{
		if (pCenterUnit == NULL)
		{
			ePlayer = GC.getGameINLINE().getActivePlayer();
		}
		else
		{
			ePlayerOffset = GC.getGameINLINE().getActivePlayer();
		}
	}

	//don't put two of the same flags
	if (ePlayerOffset == ePlayer)
	{
		ePlayerOffset = NO_PLAYER;
	}

	//destroy old flags
	if (ePlayer == NO_PLAYER || ePlayer == UNKNOWN_PLAYER)
	{
		gDLL->getFlagEntityIFace()->destroy(m_pFlagSymbol);
	}
	if (ePlayerOffset == NO_PLAYER || ePlayerOffset == UNKNOWN_PLAYER)
	{
		gDLL->getFlagEntityIFace()->destroy(m_pFlagSymbolOffset);
	}

	//create and/or update unit's flag
	if (ePlayer != NO_PLAYER && ePlayer != UNKNOWN_PLAYER)
	{
		if ((m_pFlagSymbol == NULL) || (gDLL->getFlagEntityIFace()->getPlayer(m_pFlagSymbol) != ePlayer))
		{
			if (m_pFlagSymbol != NULL)
			{
				gDLL->getFlagEntityIFace()->destroy(m_pFlagSymbol);
			}
			m_pFlagSymbol = gDLL->getFlagEntityIFace()->create(ePlayer);
			if (m_pFlagSymbol != NULL)
			{
				gDLL->getFlagEntityIFace()->setPlot(m_pFlagSymbol, this, false);
			}
		}

		if (m_pFlagSymbol != NULL)
		{
			gDLL->getFlagEntityIFace()->updateUnitInfo(m_pFlagSymbol, this, false);
		}
	}

	//create and/or update offset flag
	if (ePlayerOffset != NO_PLAYER && ePlayerOffset != UNKNOWN_PLAYER)
	{
		if ((m_pFlagSymbolOffset == NULL) || (gDLL->getFlagEntityIFace()->getPlayer(m_pFlagSymbolOffset) != ePlayerOffset))
		{
			if (m_pFlagSymbolOffset != NULL)
			{
				gDLL->getFlagEntityIFace()->destroy(m_pFlagSymbolOffset);
			}
			m_pFlagSymbolOffset = gDLL->getFlagEntityIFace()->create(ePlayerOffset);
			if (m_pFlagSymbolOffset != NULL)
			{
				gDLL->getFlagEntityIFace()->setPlot(m_pFlagSymbolOffset, this, true);
			}
		}

		if (m_pFlagSymbolOffset != NULL)
		{
			gDLL->getFlagEntityIFace()->updateUnitInfo(m_pFlagSymbolOffset, this, true);
		}
	}
}


CvUnit* CvPlot::getCenterUnit() const
{
	return m_pCenterUnit;
}


CvUnit* CvPlot::getDebugCenterUnit() const
{
	CvUnit* pCenterUnit;

	pCenterUnit = getCenterUnit();

	if (pCenterUnit == NULL)
	{
		if (GC.getGameINLINE().isDebugMode())
		{
			CLLNode<IDInfo>* pUnitNode = headUnitNode();
			if(pUnitNode == NULL)
				pCenterUnit = NULL;
			else
				pCenterUnit = ::getUnit(pUnitNode->m_data);
		}
	}

	return pCenterUnit;
}


void CvPlot::setCenterUnit(CvUnit* pNewValue)
{
	CvUnit* pOldValue;

	pOldValue = getCenterUnit();

	if (pOldValue != pNewValue)
	{
		m_pCenterUnit = pNewValue;
		updateMinimapColor();

		setFlagDirty(true);

		if (getCenterUnit() != NULL)
		{
			getCenterUnit()->setInfoBarDirty(true);
		}
	}
}


int CvPlot::getCultureRangeCities(PlayerTypes eOwnerIndex, int iRangeIndex) const
{
	FAssert(eOwnerIndex >= 0);
	FAssert(eOwnerIndex < MAX_PLAYERS);
	FAssert(iRangeIndex >= 0);
	FAssert(iRangeIndex < GC.getNumCultureLevelInfos());

	if (NULL == m_apaiCultureRangeCities)
	{
		return 0;
	}
	else if (NULL == m_apaiCultureRangeCities[eOwnerIndex])
	{
		return 0;
	}

	return m_apaiCultureRangeCities[eOwnerIndex][iRangeIndex];
}


bool CvPlot::isCultureRangeCity(PlayerTypes eOwnerIndex, int iRangeIndex) const
{
	return (getCultureRangeCities(eOwnerIndex, iRangeIndex) > 0);
}


void CvPlot::changeCultureRangeCities(PlayerTypes eOwnerIndex, int iRangeIndex, int iChange)
{
	bool bOldCultureRangeCities;

	FAssert(eOwnerIndex >= 0);
	FAssert(eOwnerIndex < MAX_PLAYERS);
	FAssert(iRangeIndex >= 0);
	FAssert(iRangeIndex < GC.getNumCultureLevelInfos());

	if (0 != iChange)
	{
		bOldCultureRangeCities = isCultureRangeCity(eOwnerIndex, iRangeIndex);

		if (NULL == m_apaiCultureRangeCities)
		{
			m_apaiCultureRangeCities = new char*[MAX_PLAYERS];
			for (int iI = 0; iI < MAX_PLAYERS; ++iI)
			{
				m_apaiCultureRangeCities[iI] = NULL;
			}
		}

		if (NULL == m_apaiCultureRangeCities[eOwnerIndex])
		{
			m_apaiCultureRangeCities[eOwnerIndex] = new char[GC.getNumCultureLevelInfos()];
			for (int iI = 0; iI < GC.getNumCultureLevelInfos(); ++iI)
			{
				m_apaiCultureRangeCities[eOwnerIndex][iI] = 0;
			}
		}

		m_apaiCultureRangeCities[eOwnerIndex][iRangeIndex] += iChange;

		if (bOldCultureRangeCities != isCultureRangeCity(eOwnerIndex, iRangeIndex))
		{
			updateCulture(true);
		}
	}
}


int CvPlot::getInvisibleVisibilityCount(TeamTypes eTeam, InvisibleTypes eInvisible) const
{
	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");
	FAssertMsg(eInvisible >= 0, "eInvisible is expected to be non-negative (invalid Index)");
	FAssertMsg(eInvisible < GC.getNumInvisibleInfos(), "eInvisible is expected to be within maximum bounds (invalid Index)");

	if (NULL == m_apaiInvisibleVisibilityCount)
	{
		return 0;
	}
	else if (NULL == m_apaiInvisibleVisibilityCount[eTeam])
	{
		return 0;
	}

	return m_apaiInvisibleVisibilityCount[eTeam][eInvisible];
}


bool CvPlot::isInvisibleVisible(TeamTypes eTeam, InvisibleTypes eInvisible)	const
{
	return (getInvisibleVisibilityCount(eTeam, eInvisible) > 0);
}


void CvPlot::changeInvisibleVisibilityCount(TeamTypes eTeam, InvisibleTypes eInvisible, int iChange)
{
	bool bOldInvisibleVisible;

	FAssertMsg(eTeam >= 0, "eTeam is expected to be non-negative (invalid Index)");
	FAssertMsg(eTeam < MAX_TEAMS, "eTeam is expected to be within maximum bounds (invalid Index)");
	FAssertMsg(eInvisible >= 0, "eInvisible is expected to be non-negative (invalid Index)");
	FAssertMsg(eInvisible < GC.getNumInvisibleInfos(), "eInvisible is expected to be within maximum bounds (invalid Index)");

	if (iChange != 0)
	{
		bOldInvisibleVisible = isInvisibleVisible(eTeam, eInvisible);

		if (NULL == m_apaiInvisibleVisibilityCount)
		{
			m_apaiInvisibleVisibilityCount = new short*[MAX_TEAMS];
			for (int iI = 0; iI < MAX_TEAMS; ++iI)
			{
				m_apaiInvisibleVisibilityCount[iI] = NULL;
			}
		}

		if (NULL == m_apaiInvisibleVisibilityCount[eTeam])
		{
			m_apaiInvisibleVisibilityCount[eTeam] = new short[GC.getNumInvisibleInfos()];
			for (int iI = 0; iI < GC.getNumInvisibleInfos(); ++iI)
			{
				m_apaiInvisibleVisibilityCount[eTeam][iI] = 0;
			}
		}

		m_apaiInvisibleVisibilityCount[eTeam][eInvisible] += iChange;

		if (bOldInvisibleVisible != isInvisibleVisible(eTeam, eInvisible))
		{
			if (eTeam == GC.getGameINLINE().getActiveTeam())
			{
				updateCenterUnit();
			}
		}
	}
}


int CvPlot::getNumUnits() const
{
	return m_units.getLength();
}


CvUnit* CvPlot::getUnitByIndex(int iIndex) const
{
	CLLNode<IDInfo>* pUnitNode;

	pUnitNode = m_units.nodeNum(iIndex);

	if (pUnitNode != NULL)
	{
		return ::getUnit(pUnitNode->m_data);
	}
	else
	{
		return NULL;
	}
}


void CvPlot::addUnit(CvUnit* pUnit, bool bUpdate)
{
	CLLNode<IDInfo>* pUnitNode;
	CvUnit* pLoopUnit;

	FAssertMsg(pUnit->at(getX_INLINE(), getY_INLINE()), "pUnit is expected to be at getX_INLINE and getY_INLINE");

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		pLoopUnit = ::getUnit(pUnitNode->m_data);

		if (!isBeforeUnitCycle(pLoopUnit, pUnit))
		{
			break;
		}

		pUnitNode = nextUnitNode(pUnitNode);
	}

	if (pUnitNode != NULL)
	{
		m_units.insertBefore(pUnit->getIDInfo(), pUnitNode);
	}
	else
	{
		m_units.insertAtEnd(pUnit->getIDInfo());
	}

	if (bUpdate)
	{
		updateCenterUnit();

		setFlagDirty(true);
	}
}


void CvPlot::removeUnit(CvUnit* pUnit, bool bUpdate)
{
	CLLNode<IDInfo>* pUnitNode;

	pUnitNode = headUnitNode();

	while (pUnitNode != NULL)
	{
		if (::getUnit(pUnitNode->m_data) == pUnit)
		{
			FAssertMsg(::getUnit(pUnitNode->m_data)->at(getX_INLINE(), getY_INLINE()), "The current unit instance is expected to be at getX_INLINE and getY_INLINE");
			m_units.deleteNode(pUnitNode);
			break;
		}
		else
		{
			pUnitNode = nextUnitNode(pUnitNode);
		}
	}

	if (bUpdate)
	{
		updateCenterUnit();

		setFlagDirty(true);
	}
}


CLLNode<IDInfo>* CvPlot::nextUnitNode(CLLNode<IDInfo>* pNode) const
{
	return m_units.next(pNode);
}


CLLNode<IDInfo>* CvPlot::prevUnitNode(CLLNode<IDInfo>* pNode) const
{
	return m_units.prev(pNode);
}


CLLNode<IDInfo>* CvPlot::headUnitNode() const
{
	return m_units.head();
}


CLLNode<IDInfo>* CvPlot::tailUnitNode() const
{
	return m_units.tail();
}


CvString CvPlot::getScriptData() const
{
	return m_szScriptData;
}

void CvPlot::setScriptData(const char* szNewValue)
{
	SAFE_DELETE_ARRAY(m_szScriptData);
	m_szScriptData = _strdup(szNewValue);
}

// Protected Functions...

void CvPlot::doFeature()
{
	PROFILE_FUNC();

	CvCity* pCity;
	CvPlot* pLoopPlot;
	CvWString szBuffer;
	int iProbability;
	int iI, iJ;

	if (getFeatureType() != NO_FEATURE)
	{
		iProbability = GC.getFeatureInfo(getFeatureType()).getDisappearanceProbability();

		if (iProbability > 0)
		{
			if (GC.getGameINLINE().getSorenRandNum(10000, "Feature Disappearance") < iProbability)
			{
				setFeatureType(NO_FEATURE);
			}
		}
	}
	else
	{
		if (!isUnit())
		{
			if (getImprovementType() == NO_IMPROVEMENT)
			{
				for (iI = 0; iI < GC.getNumFeatureInfos(); ++iI)
				{
					if (canHaveFeature((FeatureTypes)iI))
					{
						if ((getBonusType() == NO_BONUS) || (GC.getBonusInfo(getBonusType()).isFeature(iI)))
						{
							iProbability = 0;
							
							// R&R, Robert Surcouf, Damage on Storm plots, Start
							if (GC.getFeatureInfo((FeatureTypes)iI).isGeneratedEveryRound())
							{
								iProbability += GC.getFeatureInfo((FeatureTypes)iI).getAppearanceProbability();
							
								if (iProbability > 0)
								{
									if (GC.getGameINLINE().getSorenRandNum(10000, "Feature Appearance") < iProbability)
										{
											setFeatureType((FeatureTypes)iI);
										}
									iProbability = 0;
								}
							}
							// R&R, Robert Surcouf, Damage on Storm plots, End
						
							for (iJ = 0; iJ < NUM_CARDINALDIRECTION_TYPES; iJ++)
							{
								pLoopPlot = plotCardinalDirection(getX_INLINE(), getY_INLINE(), ((CardinalDirectionTypes)iJ));

								if (pLoopPlot != NULL)
								{
									if (pLoopPlot->getFeatureType() == ((FeatureTypes)iI))
									{
										if (pLoopPlot->getImprovementType() == NO_IMPROVEMENT)
										{
											iProbability += GC.getFeatureInfo((FeatureTypes)iI).getGrowthProbability();
										}
										else
										{
											iProbability += GC.getImprovementInfo(pLoopPlot->getImprovementType()).getFeatureGrowthProbability();
										}
									}
								}
							}

							iProbability *= std::max(0, (GC.getFEATURE_GROWTH_MODIFIER() + 100));
							iProbability /= 100;

							if (isRoute())
							{
								iProbability *= std::max(0, (GC.getROUTE_FEATURE_GROWTH_MODIFIER() + 100));
								iProbability /= 100;
							}

							if (iProbability > 0)
							{
								if (GC.getGameINLINE().getSorenRandNum(10000, "Feature Growth") < iProbability)
								{
									setFeatureType((FeatureTypes)iI);

									pCity = GC.getMapINLINE().findCity(getX_INLINE(), getY_INLINE(), getOwnerINLINE(), NO_TEAM, false);

									if (pCity != NULL)
									{
										// Tell the owner of this city.
										szBuffer = gDLL->getText("TXT_KEY_MISC_FEATURE_GROWN_NEAR_CITY", GC.getFeatureInfo((FeatureTypes) iI).getTextKeyWide(), pCity->getNameKey());
										gDLL->getInterfaceIFace()->addMessage(getOwnerINLINE(), false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_FEATUREGROWTH", MESSAGE_TYPE_INFO, GC.getFeatureInfo((FeatureTypes) iI).getButton(), (ColorTypes)GC.getInfoTypeForString("COLOR_WHITE"), getX_INLINE(), getY_INLINE(), true, true);
									}

									break;
								}
							}
						}
					}
				}
			}
		}
	}
}

//R&R mod, vetiarvind, super forts (outposts) Culture merge - start
void CvPlot::doCulture()
{
	PROFILE_FUNC();
		
	//CvCity* pCity;	
	CvWString szBuffer;
	PlayerTypes eCulturalOwner;
	//int iGarrison;
	//int iCityStrength;
		
	doImprovementCulture();

	ImprovementTypes eImprovement = getImprovementType();
	if(eImprovement != NO_IMPROVEMENT)
	{
		// Check for a fort culture flip
		if(GC.getImprovementInfo(eImprovement).isActsAsCity() && (getOwnershipDuration() > GC.getDefineINT("SUPER_FORTS_DURATION_BEFORE_REVOLT")))
		{
			eCulturalOwner = calculateCulturalOwner();
			if(eCulturalOwner != NO_PLAYER)
			{
				if(GET_PLAYER(eCulturalOwner).getTeam() != getTeam())
				{
					bool bDefenderFound = false;
					
					CLLNode<IDInfo>* pUnitNode = headUnitNode();
					CvUnit* pLoopUnit;
					
					while (pUnitNode != NULL)
					{
						pLoopUnit = ::getUnit(pUnitNode->m_data);
						
						pUnitNode = nextUnitNode(pUnitNode);
						if(pLoopUnit != NULL && pLoopUnit->canDefend(this)) //R&R mod, vetiarvind, super forts merge, added null check
						{
							if(pLoopUnit->getOwner() == getOwnerINLINE())
							{
								bDefenderFound = true;
								break;
							}
						}
					}
					if(!bDefenderFound)
					{
						szBuffer = gDLL->getText("TXT_KEY_MISC_CITY_REVOLTED_JOINED", GC.getImprovementInfo(getImprovementType()).getText(), GET_PLAYER(eCulturalOwner).getCivilizationDescriptionKey());
						gDLL->getInterfaceIFace()->addMessage(getOwnerINLINE(), false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CULTUREEXPANDS", MESSAGE_TYPE_INFO, GC.getImprovementInfo(getImprovementType()).getButton(), (ColorTypes)GC.getInfoTypeForString("COLOR_RED"), getX_INLINE(), getY_INLINE(), true, true);
						gDLL->getInterfaceIFace()->addMessage(eCulturalOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CULTUREEXPANDS", MESSAGE_TYPE_INFO, GC.getImprovementInfo(getImprovementType()).getButton(), (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), getX_INLINE(), getY_INLINE(), true, true);
						setOwner(eCulturalOwner,true);
					}
				}
			}
		}
	}
	
	

	updateCulture(true);
}
//R&R mod, vetiarvind, super forts (outposts) merge - end

void CvPlot::processArea(CvArea* pArea, int iChange)
{
	CvCity* pCity;
	int iI, iJ;

	// XXX am not updating getBestFoundValue() or getAreaAIType()...

	pArea->changeNumTiles(iChange);

	if (isOwned())
	{
		pArea->changeNumOwnedTiles(iChange);
	}

	if (isNOfRiver())
	{
		pArea->changeNumRiverEdges(iChange);
	}
	if (isWOfRiver())
	{
		pArea->changeNumRiverEdges(iChange);
	}

	if (getBonusType() != NO_BONUS)
	{
		pArea->changeNumBonuses(getBonusType(), iChange);
	}

	if (getImprovementType() != NO_IMPROVEMENT)
	{
		pArea->changeNumImprovements(getImprovementType(), iChange);
	}

	for (iI = 0; iI < MAX_PLAYERS; ++iI)
	{
		if (GET_PLAYER((PlayerTypes)iI).getStartingPlot() == this)
		{
			pArea->changeNumStartingPlots(iChange);
		}

		pArea->changePower(((PlayerTypes)iI), (getUnitPower((PlayerTypes)iI) * iChange));

		pArea->changeUnitsPerPlayer(((PlayerTypes)iI), (plotCount(PUF_isPlayer, iI) * iChange));

		for (iJ = 0; iJ < NUM_UNITAI_TYPES; iJ++)
		{
			pArea->changeNumAIUnits(((PlayerTypes)iI), ((UnitAITypes)iJ), (plotCount(PUF_isUnitAIType, iJ, -1, ((PlayerTypes)iI)) * iChange));
		}
	}

	for (iI = 0; iI < MAX_TEAMS; ++iI)
	{
		if (isRevealed(((TeamTypes)iI), false))
		{
			pArea->changeNumRevealedTiles(((TeamTypes)iI), iChange);
		}
	}

	pCity = getPlotCity();

	if (pCity != NULL)
	{
		pArea->changeCitiesPerPlayer(pCity->getOwnerINLINE(), iChange);
		pArea->changePopulationPerPlayer(pCity->getOwnerINLINE(), (pCity->getPopulation() * iChange));

		for (iI = 0; iI < GC.getNumBuildingInfos(); ++iI)
		{
			if (pCity->isHasBuilding((BuildingTypes)iI))
			{
				pArea->changePower(pCity->getOwnerINLINE(), (GC.getBuildingInfo((BuildingTypes)iI).getPowerValue() * iChange));
			}
		}

		for (iI = 0; iI < NUM_YIELD_TYPES; ++iI)
		{
			YieldTypes eYield = (YieldTypes) iI;
			pArea->changePower(pCity->getOwnerINLINE(), pCity->getYieldStored(eYield) * GC.getYieldInfo(eYield).getPowerValue() * iChange);
		}

		for (iI = 0; iI < NUM_UNITAI_TYPES; ++iI)
		{
			pArea->changeNumTrainAIUnits(pCity->getOwnerINLINE(), ((UnitAITypes)iI), (pCity->getNumTrainUnitAI((UnitAITypes)iI) * iChange));
		}

		for (iI = 0; iI < MAX_PLAYERS; ++iI)
		{
			if (pArea->getTargetCity((PlayerTypes)iI) == pCity)
			{
				pArea->setTargetCity(((PlayerTypes)iI), NULL);
			}
		}
	}
}


ColorTypes CvPlot::plotMinimapColor()
{
	CvUnit* pCenterUnit;

	if (GC.getGameINLINE().getActivePlayer() != NO_PLAYER)
	{
		CvCity* pCity;

		pCity = getPlotCity();

		if ((pCity != NULL) && pCity->isRevealed(GC.getGameINLINE().getActiveTeam(), true))
		{
			return (ColorTypes)GC.getInfoTypeForString("COLOR_WHITE");
		}

		if (isActiveVisible(true))
		{
			pCenterUnit = getDebugCenterUnit();

			if (pCenterUnit != NULL)
			{
				return ((ColorTypes)(GC.getPlayerColorInfo(pCenterUnit->getPlayerColor()).getColorTypePrimary()));
			}
		}

		PlayerTypes eRevealedOwner = getRevealedOwner(GC.getGameINLINE().getActiveTeam(), true);
		if (eRevealedOwner != NO_PLAYER)
		{
			if (!GET_PLAYER(eRevealedOwner).isNative())
			{
				return ((ColorTypes)(GC.getPlayerColorInfo(GET_PLAYER(getRevealedOwner(GC.getGameINLINE().getActiveTeam(), true)).getPlayerColor()).getColorTypePrimary()));
			}
		}
	}

	return (ColorTypes)GC.getInfoTypeForString("COLOR_CLEAR");
}

//
// read object from a stream
// used during load
//
void CvPlot::read(FDataStreamBase* pStream)
{
	int iI;
	bool bVal;
	char cCount;
	int iCount;

	// Init saved data
	reset();

	uint uiFlag=0;
	pStream->Read(&uiFlag);	// flags for expansion

	pStream->Read(&m_iX);
	pStream->Read(&m_iY);
	pStream->Read(&m_iArea);
	// m_pPlotArea not saved
	pStream->Read(&m_iFeatureVariety);
	pStream->Read(&m_iOwnershipDuration);
	pStream->Read(&m_iImprovementDuration);
	pStream->Read(&m_iUpgradeProgress);
	pStream->Read(&m_iForceUnownedTimer);
	pStream->Read(&m_iCityRadiusCount);
	pStream->Read(&m_iRiverID);
	pStream->Read(&m_iMinOriginalStartDist);
	pStream->Read(&m_iRiverCrossingCount);
	pStream->Read(&m_iDistanceToOcean);
	pStream->Read(&m_iCrumbs);

	// Super Forts begin *canal* *choke*
	pStream->Read(&m_iCanalValue);
	pStream->Read(&m_iChokeValue);
	// Super Forts end
	// Super Forts begin *bombard*
	pStream->Read(&m_iDefenseDamage);
	pStream->Read(&m_bBombarded);
	// Super Forts end

	pStream->Read(&bVal);
	m_bStartingPlot = bVal;
	pStream->Read(&bVal);
	m_bHills = bVal;
	pStream->Read(&bVal);
	m_bNOfRiver = bVal;
	pStream->Read(&bVal);
	m_bWOfRiver = bVal;
	pStream->Read(&bVal);
	m_bPotentialCityWork = bVal;
	// m_bShowCitySymbols not saved
	// m_bFlagDirty not saved
	// m_bPlotLayoutDirty not saved
	// m_bLayoutStateWorked not saved
	// m_bImpassable not saved

	pStream->Read(&m_eOwner);
	pStream->Read(&m_ePlotType);
	pStream->Read(&m_eTerrainType);
	pStream->Read(&m_eFeatureType);
	pStream->Read(&m_eBonusType);
	pStream->Read(&m_eImprovementType);
	pStream->Read(&m_eRouteType);
	pStream->Read(&m_eRiverNSDirection);
	pStream->Read(&m_eRiverWEDirection);
	pStream->Read(&m_eEurope);
	updateImpassable();

	m_plotCity.read(pStream);
	m_workingCity.read(pStream);
	m_workingCityOverride.read(pStream);

	pStream->Read(NUM_YIELD_TYPES, m_aiYield);

	pStream->Read(MAX_PLAYERS, m_aiDangerMap);	// TAC - AI Improved Naval AI - koma13

	SAFE_DELETE_ARRAY(m_aiCulture);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_aiCulture = new int[cCount];
		pStream->Read(cCount, m_aiCulture);
	}

	// Super Forts begin *culture*
	SAFE_DELETE_ARRAY(m_aiCultureRangeForts);
	pStream->Read(&cCount);
	if(cCount > 0)
	{
		m_aiCultureRangeForts = new short[cCount];
		pStream->Read(cCount, m_aiCultureRangeForts);
	}
	// Super Forts end

	SAFE_DELETE_ARRAY(m_aiFoundValue);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_aiFoundValue = new int[cCount];
		pStream->Read(cCount, m_aiFoundValue);
	}

	SAFE_DELETE_ARRAY(m_aiPlayerCityRadiusCount);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_aiPlayerCityRadiusCount = new char[cCount];
		pStream->Read(cCount, m_aiPlayerCityRadiusCount);
	}

	SAFE_DELETE_ARRAY(m_aiVisibilityCount);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_aiVisibilityCount = new short[cCount];
		pStream->Read(cCount, m_aiVisibilityCount);
	}

	SAFE_DELETE_ARRAY(m_aiRevealedOwner);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_aiRevealedOwner = new char[cCount];
		pStream->Read(cCount, m_aiRevealedOwner);
	}

	SAFE_DELETE_ARRAY(m_abRiverCrossing);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_abRiverCrossing = new bool[cCount];
		pStream->Read(cCount, m_abRiverCrossing);
	}

	SAFE_DELETE_ARRAY(m_abRevealed);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_abRevealed = new bool[cCount];
		pStream->Read(cCount, m_abRevealed);
	}

	SAFE_DELETE_ARRAY(m_aeRevealedImprovementType);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_aeRevealedImprovementType = new short[cCount];
		pStream->Read(cCount, m_aeRevealedImprovementType);
	}

	SAFE_DELETE_ARRAY(m_aeRevealedRouteType);
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_aeRevealedRouteType = new short[cCount];
		pStream->Read(cCount, m_aeRevealedRouteType);
	}

	m_szScriptData = pStream->ReadString();

	SAFE_DELETE_ARRAY(m_paiBuildProgress);
	pStream->Read(&iCount);
	if (iCount > 0)
	{
		m_paiBuildProgress = new short[iCount];
		pStream->Read(iCount, m_paiBuildProgress);
	}

	if (NULL != m_apaiCultureRangeCities)
	{
		for (int iI = 0; iI < MAX_PLAYERS; ++iI)
		{
			SAFE_DELETE_ARRAY(m_apaiCultureRangeCities[iI]);
		}
		SAFE_DELETE_ARRAY(m_apaiCultureRangeCities);
	}
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_apaiCultureRangeCities = new char*[cCount];
		for (iI = 0; iI < cCount; ++iI)
		{
			pStream->Read(&iCount);
			if (iCount > 0)
			{
				m_apaiCultureRangeCities[iI] = new char[iCount];
				pStream->Read(iCount, m_apaiCultureRangeCities[iI]);
			}
			else
			{
				m_apaiCultureRangeCities[iI] = NULL;
			}
		}
	}

	if (NULL != m_apaiInvisibleVisibilityCount)
	{
		for (int iI = 0; iI < MAX_TEAMS; ++iI)
		{
			SAFE_DELETE_ARRAY(m_apaiInvisibleVisibilityCount[iI]);
		}
		SAFE_DELETE_ARRAY(m_apaiInvisibleVisibilityCount);
	}
	pStream->Read(&cCount);
	if (cCount > 0)
	{
		m_apaiInvisibleVisibilityCount = new short*[cCount];
		for (iI = 0; iI < cCount; ++iI)
		{
			pStream->Read(&iCount);
			if (iCount > 0)
			{
				m_apaiInvisibleVisibilityCount[iI] = new short[iCount];
				pStream->Read(iCount, m_apaiInvisibleVisibilityCount[iI]);
			}
			else
			{
				m_apaiInvisibleVisibilityCount[iI] = NULL;
			}
		}
	}

	m_units.Read(pStream);
}

//
// write object to a stream
// used during save
//
void CvPlot::write(FDataStreamBase* pStream)
{
	uint iI;

	uint uiFlag=0;
	pStream->Write(uiFlag);		// flag for expansion

	pStream->Write(m_iX);
	pStream->Write(m_iY);
	pStream->Write(m_iArea);
	// m_pPlotArea not saved
	pStream->Write(m_iFeatureVariety);
	pStream->Write(m_iOwnershipDuration);
	pStream->Write(m_iImprovementDuration);
	pStream->Write(m_iUpgradeProgress);
	pStream->Write(m_iForceUnownedTimer);
	pStream->Write(m_iCityRadiusCount);
	pStream->Write(m_iRiverID);
	pStream->Write(m_iMinOriginalStartDist);
	pStream->Write(m_iRiverCrossingCount);
	pStream->Write(m_iDistanceToOcean);
	pStream->Write(m_iCrumbs);

	// Super Forts begin *canal* *choke*
	pStream->Write(m_iCanalValue);
	pStream->Write(m_iChokeValue);
	// Super Forts end
	// Super Forts begin *bombard*
	pStream->Write(m_iDefenseDamage);
	pStream->Write(m_bBombarded);
	// Super Forts end

	pStream->Write(m_bStartingPlot);
	pStream->Write(m_bHills);
	pStream->Write(m_bNOfRiver);
	pStream->Write(m_bWOfRiver);
	pStream->Write(m_bPotentialCityWork);
	// m_bShowCitySymbols not saved
	// m_bFlagDirty not saved
	// m_bPlotLayoutDirty not saved
	// m_bLayoutStateWorked not saved
	// m_bImpassable not saved

	pStream->Write(m_eOwner);
	pStream->Write(m_ePlotType);
	pStream->Write(m_eTerrainType);
	pStream->Write(m_eFeatureType);
	pStream->Write(m_eBonusType);
	pStream->Write(m_eImprovementType);
	pStream->Write(m_eRouteType);
	pStream->Write(m_eRiverNSDirection);
	pStream->Write(m_eRiverWEDirection);
	pStream->Write(m_eEurope);

	m_plotCity.write(pStream);
	m_workingCity.write(pStream);
	m_workingCityOverride.write(pStream);

	pStream->Write(NUM_YIELD_TYPES, m_aiYield);

	pStream->Write(MAX_PLAYERS, m_aiDangerMap);	// TAC - AI Improved Naval AI - koma13

	if (NULL == m_aiCulture)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_PLAYERS);
		pStream->Write(MAX_PLAYERS, m_aiCulture);
	}

	// Super Forts begin *culture*
	if (NULL == m_aiCultureRangeForts)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_PLAYERS);
		pStream->Write(MAX_PLAYERS, m_aiCultureRangeForts);
	}
	// Super Forts end

	if (NULL == m_aiFoundValue)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_PLAYERS);
		pStream->Write(MAX_PLAYERS, m_aiFoundValue);
	}

	if (NULL == m_aiPlayerCityRadiusCount)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_PLAYERS);
		pStream->Write(MAX_PLAYERS, m_aiPlayerCityRadiusCount);
	}

	if (NULL == m_aiVisibilityCount)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_TEAMS);
		pStream->Write(MAX_TEAMS, m_aiVisibilityCount);
	}

	if (NULL == m_aiRevealedOwner)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_TEAMS);
		pStream->Write(MAX_TEAMS, m_aiRevealedOwner);
	}

	if (NULL == m_abRiverCrossing)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)NUM_DIRECTION_TYPES);
		pStream->Write(NUM_DIRECTION_TYPES, m_abRiverCrossing);
	}

	if (NULL == m_abRevealed)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_TEAMS);
		pStream->Write(MAX_TEAMS, m_abRevealed);
	}

	if (NULL == m_aeRevealedImprovementType)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_TEAMS);
		pStream->Write(MAX_TEAMS, m_aeRevealedImprovementType);
	}

	if (NULL == m_aeRevealedRouteType)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_TEAMS);
		pStream->Write(MAX_TEAMS, m_aeRevealedRouteType);
	}

	pStream->WriteString(m_szScriptData);

	if (NULL == m_paiBuildProgress)
	{
		pStream->Write((int)0);
	}
	else
	{
		pStream->Write((int)GC.getNumBuildInfos());
		pStream->Write(GC.getNumBuildInfos(), m_paiBuildProgress);
	}

	if (NULL == m_apaiCultureRangeCities)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_PLAYERS);
		for (iI=0; iI < MAX_PLAYERS; ++iI)
		{
			if (NULL == m_apaiCultureRangeCities[iI])
			{
				pStream->Write((int)0);
			}
			else
			{
				pStream->Write((int)GC.getNumCultureLevelInfos());
				pStream->Write(GC.getNumCultureLevelInfos(), m_apaiCultureRangeCities[iI]);
			}
		}
	}

	if (NULL == m_apaiInvisibleVisibilityCount)
	{
		pStream->Write((char)0);
	}
	else
	{
		pStream->Write((char)MAX_TEAMS);
		for (iI=0; iI < MAX_TEAMS; ++iI)
		{
			if (NULL == m_apaiInvisibleVisibilityCount[iI])
			{
				pStream->Write((int)0);
			}
			else
			{
				pStream->Write((int)GC.getNumInvisibleInfos());
				pStream->Write(GC.getNumInvisibleInfos(), m_apaiInvisibleVisibilityCount[iI]);
			}
		}
	}

	m_units.Write(pStream);
}

void CvPlot::setLayoutDirty(bool bDirty)
{
	if (!GC.IsGraphicsInitialized())
	{
		return;
	}

	if (isLayoutDirty() != bDirty)
	{
		m_bPlotLayoutDirty = bDirty;

		if (isLayoutDirty() && (m_pPlotBuilder == NULL))
		{
			if (!updatePlotBuilder())
			{
				m_bPlotLayoutDirty = false;
			}
		}
	}
}

bool CvPlot::updatePlotBuilder()
{
	if (GC.IsGraphicsInitialized() && shouldUsePlotBuilder())
	{
		if (m_pPlotBuilder == NULL) // we need a plot builder... but it doesn't exist
		{
			m_pPlotBuilder = gDLL->getPlotBuilderIFace()->create();
			gDLL->getPlotBuilderIFace()->init(m_pPlotBuilder, this);
		}

		return true;
	}

	return false;
}

bool CvPlot::isLayoutDirty() const
{
	return m_bPlotLayoutDirty;
}

bool CvPlot::isLayoutStateDifferent() const
{
	bool bSame = true;
	// is worked
	bSame &= m_bLayoutStateWorked == isBeingWorked();

	// done
	return !bSame;
}

void CvPlot::setLayoutStateToCurrent()
{
	m_bLayoutStateWorked = isBeingWorked();
}

//------------------------------------------------------------------------------------------------

void CvPlot::getVisibleImprovementState(ImprovementTypes& eType, bool& bWorked)
{
	eType = NO_IMPROVEMENT;
	bWorked = false;

	if (GC.getGameINLINE().getActiveTeam() == NO_TEAM)
	{
		return;
	}

	if (getPlotCity() != NULL)
	{
		return;
	}

	eType = getRevealedImprovementType(GC.getGameINLINE().getActiveTeam(), true);

	if (eType == NO_IMPROVEMENT)
	{
		if (isActiveVisible(true))
		{
			if (isBeingWorked() && !isCity())
			{
				if (isWater())
				{
					eType = ((ImprovementTypes)(GC.getDefineINT("WATER_IMPROVEMENT")));
				}
				else
				{
					eType = ((ImprovementTypes)(GC.getDefineINT("LAND_IMPROVEMENT")));
				}
			}
		}
	}

	// worked state
	if (isActiveVisible(false) && isBeingWorked())
	{
		bWorked = true;
	}
}

void CvPlot::getVisibleBonusState(BonusTypes& eType, bool& bImproved, bool& bWorked)
{
	eType = NO_BONUS;
	bImproved = false;
	bWorked = false;

	if (GC.getGameINLINE().getActiveTeam() == NO_TEAM)
	{
		return;
	}

	if (getPlotCity() != NULL)
	{
		return;
	}
	
	if (GC.getGameINLINE().isDebugMode() || isRevealed(GC.getGameINLINE().getActiveTeam(), false))
	{
		eType = getBonusType();
	}
	// improved and worked states ...
	if (eType != NO_BONUS)
	{
		ImprovementTypes eRevealedImprovement = getRevealedImprovementType(GC.getGameINLINE().getActiveTeam(), true);

		if (eRevealedImprovement != NO_IMPROVEMENT)
		{
			bImproved = true;
			bWorked = isBeingWorked();
		}
	}
}

bool CvPlot::shouldUsePlotBuilder()
{
	bool bBonusImproved; bool bBonusWorked; bool bImprovementWorked;
	BonusTypes eBonusType;
	ImprovementTypes eImprovementType;
	getVisibleBonusState(eBonusType, bBonusImproved, bBonusWorked);
	getVisibleImprovementState(eImprovementType, bImprovementWorked);
	if(eBonusType != NO_BONUS || eImprovementType != NO_IMPROVEMENT)
	{
		return true;
	}
	return false;
}


int CvPlot::calculateMaxYield(YieldTypes eYield) const
{
	if (getTerrainType() == NO_TERRAIN)
	{
		return 0;
	}

	int iMaxYield = calculateNatureYield(eYield, NO_TEAM);

	int iImprovementYield = 0;
	for (int iImprovement = 0; iImprovement < GC.getNumImprovementInfos(); iImprovement++)
	{
		iImprovementYield = std::max(calculateImprovementYieldChange((ImprovementTypes)iImprovement, eYield, NO_PLAYER, true), iImprovementYield);
	}
	iMaxYield += iImprovementYield;

	int iRouteYield = 0;
	for (int iRoute = 0; iRoute < GC.getNumRouteInfos(); iRoute++)
	{
		iRouteYield = std::max(GC.getRouteInfo((RouteTypes)iRoute).getYieldChange(eYield), iRouteYield);
	}
	iMaxYield += iRouteYield;

	// R&R, ray, Landplot Yields - START
	if (!isWater() && !isImpassable())
	{
		int iBuildingYield = 0;
		for (int iBuilding = 0; iBuilding < GC.getNumBuildingInfos(); iBuilding++)
		{
			CvBuildingInfo& building = GC.getBuildingInfo((BuildingTypes)iBuilding);
			iBuildingYield = building.getLandPlotYieldChange(eYield);
		}
		iMaxYield += iBuildingYield;
	}
	// R&R, ray, Landplot Yields - END

	if (isWater() && !isImpassable())
	{
		int iBuildingYield = 0;
		for (int iBuilding = 0; iBuilding < GC.getNumBuildingInfos(); iBuilding++)
		{
			CvBuildingInfo& building = GC.getBuildingInfo((BuildingTypes)iBuilding);
			iBuildingYield = building.getSeaPlotYieldChange(eYield);
		}
		iMaxYield += iBuildingYield;
	}

	if (isRiver())
	{
		int iBuildingYield = 0;
		for (int iBuilding = 0; iBuilding < GC.getNumBuildingInfos(); iBuilding++)
		{
			CvBuildingInfo& building = GC.getBuildingInfo((BuildingTypes)iBuilding);
			iBuildingYield = std::max(building.getRiverPlotYieldChange(eYield), iBuildingYield);
		}
		iMaxYield += iBuildingYield;
	}

	int iExtraYieldThreshold = 0;
	for (int iTrait = 0; iTrait < GC.getNumTraitInfos(); ++iTrait)
	{
		CvTraitInfo& trait = GC.getTraitInfo((TraitTypes)iTrait);
		iExtraYieldThreshold  = std::max(trait.getExtraYieldThreshold(eYield), iExtraYieldThreshold);
	}
	if (iExtraYieldThreshold > 0 && iMaxYield > iExtraYieldThreshold)
	{
		iMaxYield += GC.getDefineINT("EXTRA_YIELD");
	}

	return iMaxYield;
}

int CvPlot::getYieldWithBuild(BuildTypes eBuild, YieldTypes eYield, bool bWithUpgrade) const
{
	int iYield = 0;

	bool bIgnoreFeature = false;
	if (getFeatureType() != NO_FEATURE)
	{
		if (GC.getBuildInfo(eBuild).isFeatureRemove(getFeatureType()))
		{
			bIgnoreFeature = true;
		}
	}

	iYield += calculateNatureYield(eYield, getTeam(), bIgnoreFeature);

	ImprovementTypes eImprovement = (ImprovementTypes)GC.getBuildInfo(eBuild).getImprovement();

	if (eImprovement != NO_IMPROVEMENT)
	{
		if (bWithUpgrade)
		{
			//in the case that improvements upgrade, use 2 upgrade levels higher for the
			//yield calculations.
			ImprovementTypes eUpgradeImprovement = (ImprovementTypes)GC.getImprovementInfo(eImprovement).getImprovementUpgrade();
			if (eUpgradeImprovement != NO_IMPROVEMENT)
			{
				ImprovementTypes eUpgradeImprovement2 = (ImprovementTypes)GC.getImprovementInfo(eUpgradeImprovement).getImprovementUpgrade();
				if (eUpgradeImprovement2 != NO_IMPROVEMENT)
				{
					eUpgradeImprovement = eUpgradeImprovement2;
				}
			}

			if ((eUpgradeImprovement != NO_IMPROVEMENT) && (eUpgradeImprovement != eImprovement))
			{
				eImprovement = eUpgradeImprovement;
			}
		}

		iYield += calculateImprovementYieldChange(eImprovement, eYield, getOwnerINLINE(), false);
	}

	RouteTypes eRoute = (RouteTypes)GC.getBuildInfo(eBuild).getRoute();
	if (eRoute != NO_ROUTE)
	{
		eImprovement = getImprovementType();
		if (eImprovement != NO_IMPROVEMENT)
		{
			for (int iI = 0; iI < NUM_YIELD_TYPES; iI++)
			{
				iYield += GC.getImprovementInfo(eImprovement).getRouteYieldChanges(eRoute, iI);
				if (getRouteType() != NO_ROUTE)
				{
					iYield -= GC.getImprovementInfo(eImprovement).getRouteYieldChanges(getRouteType(), iI);
				}
			}
		}
	}


	return iYield;
}

bool CvPlot::canTrigger(EventTriggerTypes eTrigger, PlayerTypes ePlayer) const
{
	FAssert(::isPlotEventTrigger(eTrigger));

	CvEventTriggerInfo& kTrigger = GC.getEventTriggerInfo(eTrigger);

	if (kTrigger.isOwnPlot() && getOwnerINLINE() != ePlayer)
	{
		return false;
	}

	if (kTrigger.getPlotType() != NO_PLOT)
	{
		if (getPlotType() != kTrigger.getPlotType())
		{
			return false;
		}
	}

	if (kTrigger.getNumFeaturesRequired() > 0)
	{
		bool bFoundValid = false;

		for (int i = 0; i < kTrigger.getNumFeaturesRequired(); ++i)
		{
			if (kTrigger.getFeatureRequired(i) == getFeatureType())
			{
				bFoundValid = true;
				break;
			}
		}

		if (!bFoundValid)
		{
			return false;
		}
	}

	if (kTrigger.getNumTerrainsRequired() > 0)
	{
		bool bFoundValid = false;

		for (int i = 0; i < kTrigger.getNumTerrainsRequired(); ++i)
		{
			if (kTrigger.getTerrainRequired(i) == getTerrainType())
			{
				bFoundValid = true;
				break;
			}
		}

		if (!bFoundValid)
		{
			return false;
		}
	}

	if (kTrigger.getNumImprovementsRequired() > 0)
	{
		bool bFoundValid = false;

		for (int i = 0; i < kTrigger.getNumImprovementsRequired(); ++i)
		{
			if (kTrigger.getImprovementRequired(i) == getImprovementType())
			{
				bFoundValid = true;
				break;
			}
		}

		if (!bFoundValid)
		{
			return false;
		}
	}

	if (kTrigger.getNumRoutesRequired() > 0)
	{
		bool bFoundValid = false;

		if (NULL == getPlotCity())
		{
		for (int i = 0; i < kTrigger.getNumRoutesRequired(); ++i)
		{
			if (kTrigger.getRouteRequired(i) == getRouteType())
			{
				bFoundValid = true;
				break;
			}
		}

		}

		if (!bFoundValid)
		{
			return false;
		}
	}

	if (kTrigger.isUnitsOnPlot())
	{
		bool bFoundValid = false;

		CLLNode<IDInfo>* pUnitNode = headUnitNode();

		while (NULL != pUnitNode)
		{
			CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
			pUnitNode = nextUnitNode(pUnitNode);

			if (pLoopUnit->getOwnerINLINE() == ePlayer)
			{
				if (-1 != pLoopUnit->getTriggerValue(eTrigger, this, false))
				{
					bFoundValid = true;
					break;
				}
			}
		}

		if (!bFoundValid)
		{
			return false;
		}
	}


	if (kTrigger.isPrereqEventCity() && kTrigger.getNumPrereqEvents() > 0)
	{
		bool bFoundValid = true;

		for (int iI = 0; iI < kTrigger.getNumPrereqEvents(); ++iI)
		{
			const EventTriggeredData* pTriggeredData = GET_PLAYER(ePlayer).getEventOccured((EventTypes)kTrigger.getPrereqEvent(iI));
			if (NULL == pTriggeredData || pTriggeredData->m_iPlotX != getX_INLINE() || pTriggeredData->m_iPlotY != getY_INLINE())
			{
				bFoundValid = false;
				break;
			}
		}

		if (!bFoundValid)
		{
			return false;
		}
	}


	return true;
}

bool CvPlot::canApplyEvent(EventTypes eEvent) const
{
	CvEventInfo& kEvent = GC.getEventInfo(eEvent);

	if (kEvent.getFeatureChange() > 0)
	{
		if (NO_FEATURE != kEvent.getFeature())
		{
			if (NO_IMPROVEMENT != getImprovementType() || !canHaveFeature((FeatureTypes)kEvent.getFeature()))
			{
				return false;
			}
		}
	}
	else if (kEvent.getFeatureChange() < 0)
	{
		if (NO_FEATURE == getFeatureType())
		{
			return false;
		}
	}

	if (kEvent.getImprovementChange() > 0)
	{
		if (NO_IMPROVEMENT != kEvent.getImprovement())
		{
			if (!canHaveImprovement((ImprovementTypes)kEvent.getImprovement(), getTeam()))
			{
				return false;
			}
		}
	}
	else if (kEvent.getImprovementChange() < 0)
	{
		if (NO_IMPROVEMENT == getImprovementType())
		{
			return false;
		}
	}

	if (kEvent.getRouteChange() < 0)
	{
		if (NO_ROUTE == getRouteType())
		{
			return false;
		}

		if (isCity())
		{
			return false;
		}
	}

	return true;
}

void CvPlot::applyEvent(EventTypes eEvent)
{
	CvEventInfo& kEvent = GC.getEventInfo(eEvent);

	if (kEvent.getFeatureChange() > 0)
	{
		if (NO_FEATURE != kEvent.getFeature())
		{
			setFeatureType((FeatureTypes)kEvent.getFeature());
		}
	}
	else if (kEvent.getFeatureChange() < 0)
	{
		setFeatureType(NO_FEATURE);
	}

	if (kEvent.getImprovementChange() > 0)
	{
		if (NO_IMPROVEMENT != kEvent.getImprovement())
		{
			setImprovementType((ImprovementTypes)kEvent.getImprovement());
		}
	}
	else if (kEvent.getImprovementChange() < 0)
	{
		setImprovementType(NO_IMPROVEMENT);
	}

	if (kEvent.getRouteChange() > 0)
	{
		if (NO_ROUTE != kEvent.getRoute())
		{
			setRouteType((RouteTypes)kEvent.getRoute());
		}
	}
	else if (kEvent.getRouteChange() < 0)
	{
		setRouteType(NO_ROUTE);
	}

	for (int i = 0; i < NUM_YIELD_TYPES; ++i)
	{
		int iChange = kEvent.getPlotExtraYield(i);
		if (0 != iChange)
		{
			GC.getGameINLINE().setPlotExtraYield(m_iX, m_iY, (YieldTypes)i, iChange);
		}
	}
}

bool CvPlot::canTrain(UnitTypes eUnit, bool bContinue, bool bTestVisible) const
{
	CvCity* pCity = getPlotCity();
	CvUnitInfo& kUnit = GC.getUnitInfo(eUnit);

	if (isCity())
	{
		if (kUnit.getDomainType() == DOMAIN_SEA)
		{
			if (!isWater() && !isCoastalLand(kUnit.getMinAreaSize()))
			{
				return false;
			}
		}
		else
		{
			if (area()->getNumTiles() < kUnit.getMinAreaSize())
			{
				return false;
			}
		}

		if (kUnit.getPrereqBuilding() != NO_BUILDINGCLASS)
		{
			BuildingTypes eBuilding = (BuildingTypes) GC.getCivilizationInfo(pCity->getCivilizationType()).getCivilizationBuildings(kUnit.getPrereqBuilding());
			if (NO_BUILDING == eBuilding)
			{
				return false;
			}

			if (!pCity->isHasConceptualBuilding(eBuilding))
			{
				SpecialBuildingTypes eSpecialBuilding = ((SpecialBuildingTypes)(GC.getBuildingInfo(eBuilding).getSpecialBuildingType()));
				if ((eSpecialBuilding == NO_SPECIALBUILDING) || !(GET_PLAYER(getOwnerINLINE()).isSpecialBuildingNotRequired(eSpecialBuilding)))
				{
					return false;
				}
			}
		}

		bool bValid = true;
		for (int iBuildingClass = 0; iBuildingClass < GC.getNumBuildingClassInfos(); ++iBuildingClass)
		{
			if (kUnit.isPrereqOrBuilding(iBuildingClass))
			{
				BuildingTypes eBuilding = (BuildingTypes) GC.getCivilizationInfo(pCity->getCivilizationType()).getCivilizationBuildings(iBuildingClass);
				SpecialBuildingTypes eSpecialBuilding = NO_SPECIALBUILDING;
				if (eBuilding != NO_BUILDING)
				{
					eSpecialBuilding = ((SpecialBuildingTypes)(GC.getBuildingInfo(eBuilding).getSpecialBuildingType()));
				}
				if ((eSpecialBuilding == NO_SPECIALBUILDING) || !(GET_PLAYER(getOwnerINLINE()).isSpecialBuildingNotRequired(eSpecialBuilding)))
				{
					bValid = false;
					BuildingTypes eBuilding = (BuildingTypes) GC.getCivilizationInfo(pCity->getCivilizationType()).getCivilizationBuildings(iBuildingClass);
					if (NO_BUILDING != eBuilding)
					{
						if (pCity->isHasConceptualBuilding(eBuilding) )
						{
							bValid = true;
							break;
						}
					}
				}
			}
		}

		if (!bValid)
		{
			return false;
		}
	}
	else
	{
		if (area()->getNumTiles() < kUnit.getMinAreaSize())
		{
			return false;
		}

		if (kUnit.getDomainType() == DOMAIN_SEA)
		{
			if (!isWater())
			{
				return false;
			}
		}
		else if (kUnit.getDomainType() == DOMAIN_LAND)
		{
			if (isWater())
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	return true;
}

bool CvPlot::isValidYieldChanges(UnitTypes eUnit) const
{
	FAssert(eUnit != NO_UNIT);

	if (GC.getUnitInfo(eUnit).isLandYieldChanges() != isWater())
	{
		return true;
	}

	if (GC.getUnitInfo(eUnit).isWaterYieldChanges() == isWater())
	{
		return true;
	}

	return false;
}


void CvPlot::setDistanceToOcean(int iNewValue)
{
	m_iDistanceToOcean = iNewValue;
}

int CvPlot::getDistanceToOcean() const
{
	return m_iDistanceToOcean;
}

CvPlot* CvPlot::findNearbyOceanPlot(int iRandomization)
{
    CvPlot* pOceanPlot = this;

    while (pOceanPlot->getDistanceToOcean() > 0)
    {
        CvPlot* pBestPlot = NULL;
        int iBestValue = MAX_INT;
        for (int iDirection = 0; iDirection < NUM_DIRECTION_TYPES; iDirection++)
        {
            CvPlot* pDirectionPlot = plotDirection(pOceanPlot->getX_INLINE(), pOceanPlot->getY_INLINE(), (DirectionTypes)iDirection);
            if (pDirectionPlot != NULL)
            {
				int iValue = pDirectionPlot->getDistanceToOcean() * (1000 + GC.getGame().getSorenRandNum(10 * iRandomization, "find nearby ocean plot"));
                if (iValue < iBestValue)
                {
                    iBestValue = iValue;
                    pBestPlot = pDirectionPlot;
                }
            }
        }
        FAssert(pBestPlot != NULL);
        if (pBestPlot == NULL)
        {
            return NULL;
        }
        pOceanPlot = pBestPlot;
    }
    return pOceanPlot;
}

int CvPlot::countFriendlyCulture(TeamTypes eTeam) const
{
	int iTotalCulture = 0;

	for (int iPlayer = 0; iPlayer < MAX_PLAYERS; ++iPlayer)
	{
		CvPlayer& kLoopPlayer = GET_PLAYER((PlayerTypes)iPlayer);
		if (kLoopPlayer.isAlive())
		{
			CvTeam& kLoopTeam = GET_TEAM(kLoopPlayer.getTeam());
			if (kLoopPlayer.getTeam() == eTeam || kLoopTeam.isOpenBorders(eTeam))
			{
				iTotalCulture += getCulture((PlayerTypes)iPlayer);
			}
		}
	}

	return iTotalCulture;
}

int CvPlot::getCrumbs() const
{
	return m_iCrumbs;
}

void CvPlot::addCrumbs(int iQuantity)
{
	m_iCrumbs = std::min(MAX_SHORT, m_iCrumbs + iQuantity);
}

const char* CvPlot::getResourceLayerIcon(ResourceLayerOptions eOption, CvWStringBuffer& szHelp, PlotIndicatorVisibilityFlags& eVisibilityFlag, ColorTypes& eColor) const
{
	const char* szIcon = NULL;

	switch (eOption)
	{
	case RESOURCE_LAYER_RESOURCES:
		if (isRevealed(GC.getGameINLINE().getActiveTeam(), true))
		{
			BonusTypes eBonus = getBonusType();
			if (eBonus != NO_BONUS)
			{
				szIcon = GC.getBonusInfo(eBonus).getButton();
				GAMETEXT.setBonusHelp(szHelp, eBonus, false);
				eVisibilityFlag = VISIBLE_ONSCREEN_ONLY;
				eColor = (ColorTypes) GC.getInfoTypeForString("COLOR_LIGHT_GREY");
			}
		}
		break;
	case RESOURCE_LAYER_NATIVE_TRADE:
		if (isRevealed(GC.getGameINLINE().getActiveTeam(), true))
		{
			CvCity* pCity = getPlotCity();
			if (pCity != NULL && pCity->isScoutVisited(GC.getGameINLINE().getActiveTeam()))
			{
				YieldTypes eYield = pCity->AI_getDesiredYield();
				if (eYield != NO_YIELD)
				{
					szIcon = GC.getYieldInfo(eYield).getButton();
					GAMETEXT.setYieldHelp(szHelp, *pCity, eYield);
					eVisibilityFlag = VISIBLE_ALWAYS;
					eColor = (ColorTypes) GC.getPlayerColorInfo(GET_PLAYER(pCity->getOwnerINLINE()).getPlayerColor()).getColorTypePrimary();
				}
			}
		}
		break;
	case RESOURCE_LAYER_NATIVE_TRAIN:
		if (isRevealed(GC.getGameINLINE().getActiveTeam(), true))
		{
			CvCity* pCity = getPlotCity();
			if (pCity != NULL && pCity->isScoutVisited(GC.getGameINLINE().getActiveTeam()))
			{
				UnitClassTypes eUnitClass = (UnitClassTypes) pCity->getTeachUnitClass();
				if (eUnitClass != NO_UNITCLASS)
				{
					UnitTypes eUnit = (UnitTypes) GC.getCivilizationInfo(GC.getGameINLINE().getActiveCivilizationType()).getCivilizationUnits(eUnitClass);
					if (eUnit != NO_UNIT)
					{
						szIcon = GET_PLAYER(GC.getGameINLINE().getActivePlayer()).getUnitButton(eUnit);
						GAMETEXT.setUnitHelp(szHelp, eUnit);
						eVisibilityFlag = VISIBLE_ALWAYS;
						eColor = (ColorTypes) GC.getPlayerColorInfo(GET_PLAYER(pCity->getOwnerINLINE()).getPlayerColor()).getColorTypePrimary();
					}
				}

			}
		}
		break;
	}

	return szIcon;
}

CvUnit* CvPlot::getUnitLayerUnit(UnitLayerOptionTypes eOption, CvWStringBuffer& szHelp, PlotIndicatorVisibilityFlags& eVisibilityFlag, ColorTypes& eColor, bool& bTestEnemyVisibility) const
{
	CvUnit* pUnit = NULL;
	bTestEnemyVisibility = false;

	TeamTypes eActiveTeam = GC.getGameINLINE().getActiveTeam();
	int iNumUnits = getNumUnits();
	float fPlotStrength = 0.0f;

	bool bRevealed = true;
	if (!GC.getGameINLINE().isDebugMode())		// in debug mode, you see EVERYTHING
	{
		bRevealed = isVisible(eActiveTeam, true);
	}

	if (iNumUnits > 0 && bRevealed)
	{
		CLLNode<IDInfo>* pUnitNode = headUnitNode();
		while (pUnitNode != NULL && pUnit == NULL)
		{
			CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
			pUnitNode = nextUnitNode(pUnitNode);

			if (!pLoopUnit->isInvisible(eActiveTeam, GC.getGame().isDebugMode()))
			{

				// now, is this unit of interest?
				bool bIsMilitary = !pLoopUnit->isUnarmed();
				bool bIsEnemy = pLoopUnit->isEnemy(eActiveTeam);
				bool bIsOnOurTeam = pLoopUnit->getTeam() == eActiveTeam;
				bool bOfInterest = false;

				switch (eOption)
				{
				case SHOW_ALL_MILITARY:
					if (bIsMilitary)
					{
						pUnit = pLoopUnit;
					}
					break;
				case SHOW_TEAM_MILITARY:
					if (bIsMilitary && bIsOnOurTeam)
					{
						pUnit = pLoopUnit;
					}
					break;
				case SHOW_ENEMIES:
					if (bIsMilitary && bIsEnemy)
					{
						pUnit = pLoopUnit;
					}
					break;
				case SHOW_ENEMIES_IN_TERRITORY:
					if (bIsMilitary)
					{
						pUnit = pLoopUnit;
					}
					break;
				case SHOW_PLAYER_DOMESTICS:
					if (!bIsMilitary)
					{
						pUnit = pLoopUnit;
					}
					break;
				default:
					break;
				}
			}
		}

		if (pUnit != NULL)
		{
			pUnit = getBestDefender(pUnit->getOwnerINLINE());
			if (pUnit != NULL)
			{
				if (eOption == SHOW_ENEMIES_IN_TERRITORY)
				{
					eColor = (ColorTypes) GC.getInfoTypeForString("COLOR_RED");
				}
				else
				{
					eColor = (ColorTypes) GC.getPlayerColorInfo(pUnit->getPlayerColor()).getColorTypePrimary();
				}
				szHelp.clear();
				GAMETEXT.setPlotListHelp(szHelp, this, true, true);

				//setup visibility
				switch (eOption)
				{
				case SHOW_ENEMIES_IN_TERRITORY:
					bTestEnemyVisibility = true;
					eVisibilityFlag = VISIBLE_ALWAYS;
					break;
				case SHOW_ENEMIES:
					eVisibilityFlag = VISIBLE_ALWAYS;
					break;
				default:
					eVisibilityFlag = VISIBLE_ONSCREEN_ONLY;
					break;
				}
			}
		}
	}

	return pUnit;
}

void CvPlot::updateImpassable()
{
	if (getTerrainType() == NO_TERRAIN)
	{
		m_bImpassable = false;
	}
	else
	{
		m_bImpassable = ((getFeatureType() == NO_FEATURE) ? GC.getTerrainInfo(getTerrainType()).isImpassable() : GC.getFeatureInfo(getFeatureType()).isImpassable());
	}
}

//R&R mod, vetiarvind, super forts merge, refactor checks for activating monastery and forts - start
CvUnit* CvPlot::getFortDefender()
{
	if (!isFort())
	{
		return NULL;
	}
		
	CLLNode<IDInfo>* pUnitNode = headUnitNode();
	CvUnit* pDefenseUnit = NULL;
	while (pUnitNode != NULL)
	{
		CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
		if (pLoopUnit == NULL)
			continue;

		const UnitCombatTypes eSiegeType = (UnitCombatTypes)GC.getInfoTypeForString("UNITCOMBAT_SIEGE");
		const UnitCombatTypes eGunType = (UnitCombatTypes)GC.getInfoTypeForString("UNITCOMBAT_GUN");

		// Erik: Non-invisible units with firearms or defensive artillery may defend the fort.
		if (!pLoopUnit->alwaysInvisible()) 
		{	
			if (pLoopUnit->getUnitCombatType() == eSiegeType && !pLoopUnit->noDefensiveBonus() || pLoopUnit->getUnitCombatType() == eGunType)
			{
				if (pLoopUnit->getTeam() == getTeam() && pLoopUnit->getFortifyTurns() > 0)
				{
					// Erik: Find the strongest valid defender
					if (pDefenseUnit == NULL || pDefenseUnit->baseCombatStr() < pLoopUnit->baseCombatStr())
					{
						pDefenseUnit = pLoopUnit;
					}
				}
			}
		}
	
		pUnitNode = nextUnitNode(pUnitNode);
	}
	return pDefenseUnit;
}

CvUnit* CvPlot::getMonasteryMissionary()
{
	if (!isMonastery())
	{
		return NULL;
	}

	bool hasMonasteryMissionary = false;
	bool hasExpertMissionary = false;
	PlayerTypes MonasteryOwner = NO_PLAYER;
	CvUnit* missionaryUnit = NULL;

	CLLNode<IDInfo>* pUnitNode = headUnitNode();
	while (pUnitNode != NULL)
	{
		CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if (pLoopUnit != NULL && pLoopUnit->getProfession() != NO_PROFESSION && GC.getProfessionInfo(pLoopUnit->getProfession()).getMissionaryRate() > 0 && pLoopUnit->getFortifyTurns() > 0 )
		{
			missionaryUnit = pLoopUnit;
			if(missionaryUnit->getUnitInfo().getMissionaryRateModifier() > 0)
				return pLoopUnit;				
		}
	}
	return missionaryUnit;
}


//R&R mod, vetiarvind, super forts merge, refactor checks for activating monastery and forts - end



// R&R, ray, Monasteries and Forts - START
void CvPlot::doFort()
{
	//R&R mod, vetiarvind, super forts merge, refactor checks for activating monastery and forts - start
	CvUnit* pDefenseUnit = getFortDefender();
	if(pDefenseUnit == NULL)
		return;	
	PlayerTypes FortOwner = pDefenseUnit->getOwner();
	/* -- original
	if (!isFort())
	{
		return;
	}

	bool hasFortDefender = false;
	PlayerTypes FortOwner = NO_PLAYER;
	CLLNode<IDInfo>* pUnitNode = headUnitNode();
	CvUnit* pDefenseUnit = NULL;
	while (pUnitNode != NULL)
	{
		CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);
		// R&R mod, vetiarvind, monasteries and forts upgrade bug fix - start		
		//if (pLoopUnit != NULL && (pLoopUnit->getUnitClassType() == GC.getDefineINT("UNITCLASS_KING_REINFORCEMENT_LAND") || pLoopUnit->getUnitClassType() == GC.getDefineINT("UNITCLASS_CONTINENTAL_GUARD") || (pLoopUnit->getProfession()!=NO_PROFESSION && GC.getProfessionInfo(pLoopUnit->getProfession()).isCityDefender() && !GC.getProfessionInfo(pLoopUnit->getProfession()).isUnarmed())) && pLoopUnit->getFortifyTurns() > 0 )
		//has fortified and not a pioneer as we don't want working pioneers to be considered fortified
		if (pLoopUnit != NULL && pLoopUnit->getFortifyTurns() > 0 && !(pLoopUnit->getProfession() != NO_PROFESSION && GC.getProfessionInfo(pLoopUnit->getProfession()).getWorkRate() != 0))		
		{
			hasFortDefender = true;
			//FortOwner = pLoopUnit->getOwner();
			
			if(pDefenseUnit == NULL || (FortOwner == pLoopUnit->getOwner() && pDefenseUnit->baseCombatStr() < pLoopUnit->baseCombatStr()))// R&R mod, vetiarvind, monasteries and forts upgrade bug fix
			{
				pDefenseUnit = pLoopUnit;		
				FortOwner = pLoopUnit->getOwner();
			}
			//pDefenseUnit = pLoopUnit;			
			//break; 
		}
		// R&R mod, vetiarvind, monasteries and forts upgrade bug fix - end
		
	}
		
	
	if (!hasFortDefender)
	{
		return;
	}
	
	doUpgradeNonWorkerImprovements(); 		// R&R mod, vetiarvind, monasteries and forts upgrade bug fix		
	*/
	//R&R mod, vetiarvind, super forts merge, refactor checks for activating monastery and forts - end
	
	//doImprovementUpgrade();  // R&R mod, vetiarvind, monasteries and forts upgrade bug fix
	
	bool alreadyFired = false;

	int defenseDamageMultiplier = GC.getDefineINT("DEFENSE_DAMAGE_MULTIPLIER");
	CvPlot* pAdjacentPlot = NULL;
	CLLNode<IDInfo>* pUnitNode2 = NULL;

	for (int iI = 0; iI < NUM_DIRECTION_TYPES; iI++)
	{
		if (alreadyFired) // R&R, ray, fix to endless loop
		{
			break;
		}
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));
		//we do not fire on Water here
		if (pAdjacentPlot != NULL && !pAdjacentPlot->isWater())
		{
			pUnitNode2 = pAdjacentPlot->headUnitNode();
			CvUnit* pLoopUnit2 = NULL;
			CvWString szBuffer;
			while (pUnitNode2)
			{
				pLoopUnit2 = ::getUnit(pUnitNode2->m_data);
				pUnitNode2 = pAdjacentPlot->nextUnitNode(pUnitNode2);

				if (pDefenseUnit->getTeam() != pLoopUnit2->getTeam() && (GET_TEAM(pDefenseUnit->getTeam()).isAtWar(pLoopUnit2->getTeam()) || pLoopUnit2->getUnitInfo().isHiddenNationality()) && !pLoopUnit2->isCargo())
				{
					int defenseRandNum = GC.getGameINLINE().getSorenRandNum(100, "Fort Defense Attack");
					int iDamage = 0;													

					// R&R mod, vetiarvind, monasteries and forts balancing: account defender relative strength in miss % calculation - start
					int chanceOfHit = defenseRandNum;
					int iAttackerCombatMod = pDefenseUnit->baseCombatStr();
					if(iAttackerCombatMod == 0)
						iAttackerCombatMod = 1;
					int iDefenderCombatMod = pLoopUnit2->baseCombatStr();
					if (iDefenderCombatMod == 0)							
						iDefenderCombatMod = 1;
						
					//additional 0-20% chance of miss for stronger defender
					int delta = 20 * (iDefenderCombatMod - iAttackerCombatMod) / iDefenderCombatMod;
					if(delta > 0)
						chanceOfHit -= delta;
					if (chanceOfHit > 10)
					//if (defenseRandNum > 10) //10 percent chance to miss						
					// R&R mod, vetiarvind, monasteries and forts balancing: account defender relative strength in miss % calculation - end						
					{
						iDamage = pLoopUnit2->maxHitPoints()* defenseRandNum / 100;
						iDamage = iDamage * defenseDamageMultiplier;

						//taking into account different strengths of units
						/*int iDefenderCombatMod = pLoopUnit2->baseCombatStr();
						if (iDefenderCombatMod == 0)
						{
							iDefenderCombatMod = 1;
						}*/

						iDamage = iDamage / iDefenderCombatMod;

						//checking Terrain
						int iTerrainDamageMod = 0;

						if (pAdjacentPlot->isPeak())
						{
							iTerrainDamageMod = 50;
						}
						else if (pAdjacentPlot->isHills())
						{
							iTerrainDamageMod = 25;
						}

						//Forests, Jungle ...
						if (pAdjacentPlot->getTerrainType() != NO_TERRAIN)
						{
							iTerrainDamageMod = iTerrainDamageMod + 20;
						}

						if (pLoopUnit2->getUnitInfo().isAnimal())
						{
							iDamage = pLoopUnit2->maxHitPoints();
							pLoopUnit2->changeDamage(iDamage, pDefenseUnit);

							int iGoldRewardRandomBase = GC.getWILD_ANIMAL_REWARD_RANDOM_BASE();
							int iGold = GC.getGameINLINE().getSorenRandNum(iGoldRewardRandomBase, "Animal Kill Reward");
							GET_PLAYER(FortOwner).changeGold(iGold);

							szBuffer = gDLL->getText("TXT_KEY_FORT_IMPROVEMENT_KILLED_ANIMAL", iGold);
							gDLL->getInterfaceIFace()->addMessage(FortOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), getX_INLINE(), getY_INLINE(), true, true);
						}

						else
						{
							iDamage = iDamage * (100 - iTerrainDamageMod) / 100;
							pLoopUnit2->changeDamage(iDamage, pDefenseUnit);
									
							if (iDamage > 0)
							{
								if (pLoopUnit2->isDead())
								{
									int iExperience = pLoopUnit2->attackXPValue();
									iExperience = range(iExperience, GC.getDefineINT("MIN_EXPERIENCE_PER_COMBAT"), GC.getDefineINT("MAX_EXPERIENCE_PER_COMBAT"));
									pDefenseUnit->changeExperience(iExperience, pLoopUnit2->maxXPValue(), true, false, true);

									szBuffer = gDLL->getText("TXT_KEY_FORT_IMPROVEMENT_DESTROYED_GOOD");
									gDLL->getInterfaceIFace()->addMessage(FortOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), getX_INLINE(), getY_INLINE(), true, true);

									szBuffer = gDLL->getText("TXT_KEY_FORT_IMPROVEMENT_DESTROYED_BAD");
									gDLL->getInterfaceIFace()->addMessage(pLoopUnit2->getOwnerINLINE(), false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_RED"), pLoopUnit2->getX_INLINE(), pLoopUnit2->getY_INLINE(), true, true);
								}
								else
								{
									szBuffer = gDLL->getText("TXT_KEY_FORT_IMPROVEMENT_HIT_LAND_GOOD");
									gDLL->getInterfaceIFace()->addMessage(FortOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), getX_INLINE(), getY_INLINE(), true, true);

									szBuffer = gDLL->getText("TXT_KEY_FORT_IMPROVEMENT_HIT_LAND_BAD");
									gDLL->getInterfaceIFace()->addMessage(pLoopUnit2->getOwnerINLINE(), false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_RED"), pLoopUnit2->getX_INLINE(), pLoopUnit2->getY_INLINE(), true, true);
								}
							}
							else
							{
								szBuffer = gDLL->getText("TXT_KEY_FORT_IMPROVEMENT_MISS_LAND_BAD");
								gDLL->getInterfaceIFace()->addMessage(FortOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_RED"), getX_INLINE(), getY_INLINE(), true, true);

								szBuffer = gDLL->getText("TXT_KEY_FORT_IMPROVEMENT_MISS_LAND_GOOD");
								gDLL->getInterfaceIFace()->addMessage(pLoopUnit2->getOwnerINLINE(), false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), pLoopUnit2->getX_INLINE(), pLoopUnit2->getY_INLINE(), true, true);
							}
						}
					}

					if (pAdjacentPlot->isActiveVisible(false))
					{
						// Bombard pDefenseUnit mission
						CvMissionDefinition kDefiniton;
						kDefiniton.setMissionTime(GC.getMissionInfo(MISSION_BOMBARD).getTime() * gDLL->getSecsPerTurn());
						kDefiniton.setMissionType(MISSION_BOMBARD);
						kDefiniton.setPlot(pAdjacentPlot);
						kDefiniton.setUnit(BATTLE_UNIT_ATTACKER, pDefenseUnit);
						kDefiniton.setUnit(BATTLE_UNIT_DEFENDER, pLoopUnit2);
						gDLL->getEntityIFace()->AddMission(&kDefiniton);
					}

					if (pLoopUnit2->isDead())
					{
						pLoopUnit2->kill(false);
					}
				}
				// we stop, because we have already fired once. reason is balancing
				pUnitNode2 = NULL;
				alreadyFired = true;
			}
		}
	}
	
}

void CvPlot::doMonastery()
{
	//R&R mod, vetiarvind, super forts merge, refactor checks for activating monastery and forts - start
	
	CvUnit* missionaryUnit = getMonasteryMissionary();
	if(missionaryUnit == NULL)
		return;
	
	bool hasExpertMissionary = missionaryUnit->getUnitInfo().getMissionaryRateModifier() > 0;
	PlayerTypes MonasteryOwner = missionaryUnit->getOwner();
	
	/* -- original
	if (!isMonastery())
	{
		return;
	}

	bool hasMonasteryMissionary = false;
	bool hasExpertMissionary = false;
	PlayerTypes MonasteryOwner = NO_PLAYER;
	CvUnit* missionaryUnit = NULL;

	CLLNode<IDInfo>* pUnitNode = headUnitNode();
	while (pUnitNode != NULL)
	{
		CvUnit* pLoopUnit = ::getUnit(pUnitNode->m_data);
		pUnitNode = nextUnitNode(pUnitNode);

		if (pLoopUnit != NULL && pLoopUnit->getProfession() != NO_PROFESSION && GC.getProfessionInfo(pLoopUnit->getProfession()).getMissionaryRate() > 0 && pLoopUnit->getFortifyTurns() > 0 )
		{
			hasMonasteryMissionary = true;
			MonasteryOwner = pLoopUnit->getOwner();
			missionaryUnit = pLoopUnit;
			if (pLoopUnit->getUnitInfo().getMissionaryRateModifier() > 0)
			{
				hasExpertMissionary = true;
			}
			break;
		}
	}
	
	if (!hasMonasteryMissionary)
	{
		return;
	}
	
	doUpgradeNonWorkerImprovements(); // R&R mod, vetiarvind, monasteries and forts upgrade bug fix
	*/
	
	
	//R&R mod, vetiarvind, super forts merge, refactor checks for activating monastery and forts - end
	
	bool alreadyDoneMonastery = false;


	CvPlot* pAdjacentPlot = NULL;
	CLLNode<IDInfo>* pUnitNode2 = NULL;

	int iChanceForConverting = GC.getDefineINT("BASE_CHANCE_MONASTERY_FEATURES");
	if (hasExpertMissionary)
	{
		iChanceForConverting = iChanceForConverting * 2;
	}
	int iChanceForPresents = iChanceForConverting * 2;
	int iChanceForApeasingRaid = iChanceForPresents * 2;
	int iAdjacentChanceForConverting = iChanceForConverting / 2; // R&R mod, vetiarvind, monasteries and forts balancing:  -50% chance for adj. tiles, +25% chance for center tile
	for (int iI = 0; iI < NUM_DIRECTION_TYPES; iI++)
	{
		if (alreadyDoneMonastery) // R&R, ray, fix to endless loop
		{
			break;
		}

		int iUsedChanceForConverting = ((DirectionTypes)iI == NO_DIRECTION) ? (iChanceForConverting * 5 / 4) : iAdjacentChanceForConverting; // R&R mod, vetiarvind, monasteries and forts balancing -50% chance for adj. tiles, +25% chance for center tile
		pAdjacentPlot = plotDirection(getX_INLINE(), getY_INLINE(), ((DirectionTypes)iI));
		//nothing on Water
		if (pAdjacentPlot != NULL && !pAdjacentPlot->isWater())
		{
			pUnitNode2 = pAdjacentPlot->headUnitNode();
			CvUnit* pLoopUnit2 = NULL;
			CvWString szBuffer;
			while (pUnitNode2)
			{
				pLoopUnit2 = ::getUnit(pUnitNode2->m_data);
				pUnitNode2 = pAdjacentPlot->nextUnitNode(pUnitNode2);

				// Native Unit and we are not at war
				if (pLoopUnit2->isNative() && !GET_TEAM(missionaryUnit->getTeam()).isAtWar(pLoopUnit2->getTeam()))
				{
					int iMonasteryFeatureChance= GC.getGameINLINE().getSorenRandNum(100, "Monastery Feature");

					if (pLoopUnit2->AI_getUnitAIState() == UNITAI_STATE_RAIDING_PARTY)
					{
						if (iChanceForApeasingRaid > iMonasteryFeatureChance)
						{
							pLoopUnit2->AI_setUnitAIState(UNITAI_STATE_WANDER);
							szBuffer = gDLL->getText("TXT_KEY_MONASTERY_IMPROVEMENT_APEASED_RAIDING_NATIVE");
							gDLL->getInterfaceIFace()->addMessage(MonasteryOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), getX_INLINE(), getY_INLINE(), true, true);
							// we stop, because we have did something. reason is balancing
							pUnitNode2 = NULL;
							alreadyDoneMonastery = true;
						}
					}
					//else if (pLoopUnit2->AI_getUnitAIState() == UNITAI_STATE_WANDER)
					else
					{						
						// R&R mod, vetiarvind, monasteries and forts balancing - start
						if (iUsedChanceForConverting > iMonasteryFeatureChance) 
						//if (iChanceForConverting > iMonasteryFeatureChance)
						// R&R mod, vetiarvind, monasteries and forts balancing - end
						{
							UnitClassTypes eUnitClass = (UnitClassTypes) GC.getCivilizationInfo(GET_PLAYER(pLoopUnit2->getOwner()).getCivilizationType()).getCapturedCityUnitClass();
							if (eUnitClass != NO_UNITCLASS)
							{
								UnitTypes eUnit = (UnitTypes) GC.getCivilizationInfo(GET_PLAYER(MonasteryOwner).getCivilizationType()).getCivilizationUnits(eUnitClass);
								if (eUnit != NO_UNIT)
								{
									CvUnit* pUnit = GET_PLAYER(MonasteryOwner).initUnit(eUnit, (ProfessionTypes) GC.getUnitInfo(eUnit).getDefaultProfession(), getX_INLINE(), getY_INLINE());
									szBuffer = gDLL->getText("TXT_KEY_MONASTERY_IMPROVEMENT_CONVERTED_NATIVE");
									gDLL->getInterfaceIFace()->addMessage(MonasteryOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), getX_INLINE(), getY_INLINE(), true, true);
								}
							}
							pLoopUnit2->kill(false);
							// we stop, because we have did something. reason is balancing
							pUnitNode2 = NULL;
							alreadyDoneMonastery = true;
						}
						else if (iChanceForPresents > iMonasteryFeatureChance)
						{
							int iGoldRewardRandomBase = GC.getDefineINT("MONASTERY_PRESENT_BASE");
							int iGold = GC.getGameINLINE().getSorenRandNum(iGoldRewardRandomBase, "Monastery Present");
							GET_PLAYER(MonasteryOwner).changeGold(iGold);
							pLoopUnit2->AI_setUnitAIState(UNITAI_STATE_RETURN_HOME);
							szBuffer = gDLL->getText("TXT_KEY_MONASTERY_IMPROVEMENT_GOT_PRESENT", iGold);
							gDLL->getInterfaceIFace()->addMessage(MonasteryOwner, false, GC.getEVENT_MESSAGE_TIME(), szBuffer, "AS2D_CIVIC_ADOPT", MESSAGE_TYPE_MINOR_EVENT, NULL, (ColorTypes)GC.getInfoTypeForString("COLOR_GREEN"), getX_INLINE(), getY_INLINE(), true, true);
							// we stop, because we have did something. reason is balancing
							pUnitNode2 = NULL;
							alreadyDoneMonastery = true;
						}
					}
				}
			}
		}
	}
}
// R&R, ray, Monasteries and Forts - END

void CvPlot::writeDesyncLog(FILE *f)
{
	fprintf(f, "\t(%d,%d)\n", this->getX_INLINE(), this->getY_INLINE());
	fprintf(f, "\tOwner: %d\n", this->getOwnerINLINE());
	for (int i = 0; i < MAX_PLAYERS; ++i)
	{
		if (GET_PLAYER((PlayerTypes)i).isAlive())
		{
			int iCulture = getCulture((PlayerTypes)i);
			if (iCulture > 0)
			{
				fprintf(f, "\tCulture player %d: %d\n", i, iCulture);
			}
		}
	}
}
