#include <Havok.h>
#include <MathUtils.h>
#include <SimpleIni.h>
#include <Utilities.h>
#include <Windows.h>
#include <unordered_map>
#include <vector>
using namespace F4SE;
#define MATH_PI 3.14159265358979323846
#define HAVOKTOFO4 69.99124f

using std::vector;
/*using RE::NiAVObject;
using RE::NiNode;
using RE::NiPointer;
using RE::NiPoint3;
using RE::NiMatrix3;
using RE::Actor;
using RE::PlayerCharacter;
using RE::PlayerCamera;
using RE::PlayerControls;
using RE::ActorValueInfo;
using RE::bhkCharacterController;
using RE::PlayerInputHandler;
using RE::PlayerControlsData;
using RE::MouseMoveEvent;
using RE::ThumbstickEvent;
using RE::UI;
using RE::CameraState;
using RE::bhkCharacterMoveFinishEvent;
using RE::BSTEventSource;
using RE::INTERACTING_STATE;
using RE::BSEventNotifyControl;
using RE::BSInputEventReceiver;
using RE::ButtonEvent;
using RE::InputEvent;
using RE::INPUT_DEVICE;
using RE::INPUT_EVENT_TYPE;
using RE::BSTEventSink;
using RE::TESForm;
using RE::GameSettingCollection;*/
using namespace RE;

#pragma region Variables

class CharacterMoveFinishEventWatcher;

REL::Relocation<uintptr_t> ptr_ADS_DistanceCheck{ REL::ID(1278889), 0x31 };
REL::Relocation<uintptr_t> ptr_PCUpdateMainThread{ REL::ID(633524), 0x22D };
uintptr_t PCUpdateMainThreadOrig;
REL::Relocation<uintptr_t> ptr_UpdateSceneGraph{ REL::ID(1318162), 0xD5 };
static uintptr_t UpdateSceneGraphOrig;
vector<hkVector4f> cachedShape[5];
CSimpleIniA ini(true, true, false);
PlayerCharacter* p;
PlayerCamera* pcam;
PlayerControls* pc;
NiNode* lastRoot;
BSBound* bbx;
ActorValueInfo* rightAttackCondition;
ActorValueInfo* leftAttackCondition;
BGSProjectile* colCheckProj;
TESQuest* MQ102;
float rotX, rotY, rotZ, transZ, targetRotX, targetRotZ;
NiMatrix3 lastCamRot;
float rotLimitX = 10.0f;
float rotLimitY = 5.0f;
float rotDivX = 10.0f;
float rotDivY = 20.0f;
float rotDivXADS = 4.0f;
float rotDivYADS = 8.0f;
float rotADSConditionMult = 5.0f;
float rotReturnDiv = 1.5f;
float rotReturnStep = 0.1f;
float rotReturnDivMin = 1.02f;
bool rotDisableInADS = false;
float lastRun;
float playerLastLoaded;
int leanState = 0;
int lastLeanState = 0;
float leanWeight;
float leanTime;
float leanTimeCost = 1.0f;
float leanMax = 15.0f;
float leanMax3rd = 30.0f;
float leanCollisionThreshold = 0.08f;
float leanDistMoved;
float leanCumulativeMouseX = 0;
float leanCumulativeMouseY = 0;
NiPoint3 leanLastDir;
bool toggleLean = false;
bool disableLean = false;
bool vanillaPeekPatched = false;
uint32_t leanLeft = 0x51;
uint32_t leanRight = 0x45;
bool leanADSOnly = false;
bool leanR6Style = false;
bool collisionDevMode = false;
bool buttonDevMode = false;
bool iniDevMode = false;
float lastiniUpdate;
float lastHeight = 120.0f;
float heightDiffThreshold = 5.0f;
float heightBuffer = 16.0f;
bool dynamicHeight = false;
bool isLoading = false;

#pragma endregion

#pragma region Utilities

void NiSetParent(NiAVObject* node, NiNode* parent)
{
	NiNode* oldParent = node->parent;
	if (oldParent) {
		oldParent->DetachChild(node);
	}
	node->parent = parent;
}

NiNode* InsertBone(NiAVObject* root, NiNode* node, const char* name)
{
	NiNode* parent = node->parent;
	NiNode* inserted = (NiNode*)root->GetObjectByName(name);
	if (!inserted) {
		inserted = CreateBone(name);
		//_MESSAGE("%s (%llx) created.", name, inserted);
		if (parent) {
			parent->DetachChild(node);
			parent->AttachChild(inserted, true);
			inserted->parent = parent;
		} else {
			parent = node;
		}
		inserted->local.translate = NiPoint3();
		inserted->local.rotate.MakeIdentity();
		inserted->AttachChild(node, true);
		//_MESSAGE("%s (%llx) inserted to %s (%llx).", name, inserted, parent->name.c_str(), parent);
		return inserted;
	} else {
		if (!inserted->GetObjectByName(node->name)) {
			_MESSAGE("%s structure mismatch. Reinserting...", inserted->name.c_str());
			inserted->parent->DetachChild(inserted);
			inserted = CreateBone(name);
			//_MESSAGE("%s (%llx) created.", name, inserted);
			if (parent) {
				parent->DetachChild(node);
				parent->AttachChild(inserted, true);
				inserted->parent = parent;
			} else {
				parent = node;
			}
			inserted->local.translate = NiPoint3();
			inserted->local.rotate.MakeIdentity();
			inserted->AttachChild(node, true);
		}
	}
	return nullptr;
}

bool IsInPowerArmor(Actor* a)
{
	if (!a->extraList) {
		return false;
	}
	return a->extraList->HasType(RE::EXTRA_DATA_TYPE::kPowerArmor);
	;
}

bool IsInADS(Actor* a)
{
	return ((int)a->gunState == 0x8 || (int)a->gunState == 0x6);
}

bool IsFirstPerson()
{
	return pcam->currentState == pcam->cameraStates[CameraState::kFirstPerson];
}

float Sign(float f)
{
	if (f == 0)
		return 0;
	return abs(f) / f;
}

enum NiAVObjectFlag
{
	kNone = 0,
	kHidden = 1 << 0,
	kSelectiveUpdate = 1 << 1,
	kSelectiveUpdateTransforms = 1 << 2,
	kSelectiveUpdateController = 1 << 3,
	kSelectiveUpdateRigid = 1 << 4,
	kDisplayObject = 1 << 5,
	kDisableSorting = 1 << 6,
	kSelectiveUpdateTransformsOverride = 1 << 7,
	kSaveExternalGeometryData = 1 << 9,
	kNoDecals = 1 << 10,
	kAlwaysDraw = 1 << 11,
	kMeshLOD = 1 << 12,
	kFixedBound = 1 << 13,
	kTopFadeNode = 1 << 14,
	kIgnoreFade = 1 << 15,
	kNoAnimSyncX = 1 << 16,
	kNoAnimSyncY = 1 << 17,
	kNoAnimSyncZ = 1 << 18,
	kNoAnimSyncS = 1 << 19,
	kNoDismember = 1 << 20,
	kNoDismemberValidity = 1 << 21,
	kRenderUse = 1 << 22,
	kMaterialsApplied = 1 << 23,
	kHighDetail = 1 << 24,
	kForceUpdate = 1 << 25,
	kPreProcessedNode = 1 << 26
};

#pragma endregion

#pragma region Functions

void LoadConfigs()
{
	ini.LoadFile("Data\\F4SE\\Plugins\\UneducatedShooter.ini");
	rotLimitX = std::stof(ini.GetValue("Inertia", "rotLimitX", "16.0"));
	rotLimitY = std::stof(ini.GetValue("Inertia", "rotLimitY", "8.0"));
	rotDivX = std::stof(ini.GetValue("Inertia", "rotDivX", "4.0"));
	rotDivY = std::stof(ini.GetValue("Inertia", "rotDivY", "4.0"));
	rotDivXADS = std::stof(ini.GetValue("Inertia", "rotDivXADS", "12.0"));
	rotDivYADS = std::stof(ini.GetValue("Inertia", "rotDivYADS", "12.0"));
	rotADSConditionMult = std::stof(ini.GetValue("Inertia", "rotADSConditionMult", "4.0"));
	rotReturnDiv = std::stof(ini.GetValue("Inertia", "rotReturnDiv", "2.0"));
	rotReturnDivMin = std::stof(ini.GetValue("Inertia", "rotReturnDivMin", "1.05"));
	rotReturnStep = std::stof(ini.GetValue("Inertia", "rotReturnStep", "0.0"));
	rotDisableInADS = std::stoi(ini.GetValue("Inertia", "rotDisableInADS", "0")) > 0;
	disableLean = std::stoi(ini.GetValue("Leaning", "leanDisable", "0")) > 0;
	leanTimeCost = std::stof(ini.GetValue("Leaning", "leanTimeCost", "1.0"));
	leanMax = std::stof(ini.GetValue("Leaning", "leanMax", "15.0"));
	leanMax3rd = std::stof(ini.GetValue("Leaning", "leanMax3rd", "30.0"));
	toggleLean = std::stoi(ini.GetValue("Leaning", "ToggleLean", "0")) > 0;
	leanLeft = std::stoi(ini.GetValue("Leaning", "LeanLeft", "0x51"), 0, 16);
	leanRight = std::stoi(ini.GetValue("Leaning", "LeanRight", "0x45"), 0, 16);
	leanADSOnly = std::stoi(ini.GetValue("Leaning", "ADSOnly", "0")) > 0;
	leanR6Style = std::stoi(ini.GetValue("Leaning", "R6Style", "0")) > 0;
	dynamicHeight = std::stoi(ini.GetValue("Height", "dynamicHeight", "1")) > 0;
	heightDiffThreshold = std::stof(ini.GetValue("Height", "heightDiffThreshold", "5.0"));
	heightBuffer = std::stof(ini.GetValue("Height", "heightBuffer", "16.0"));
	leanCollisionThreshold = std::stof(ini.GetValue("Dev", "leanCollisionThreshold", "0.08"));
	collisionDevMode = std::stoi(ini.GetValue("Dev", "collisionDevMode", "0")) > 0;
	buttonDevMode = std::stoi(ini.GetValue("Dev", "buttonDevMode", "0")) > 0;
	iniDevMode = std::stoi(ini.GetValue("Dev", "iniDevMode", "0")) > 0;
	ini.Reset();

	if (!vanillaPeekPatched && !disableLean) {
		uint8_t bytes[] = { 0xE9, 0x1C, 0x06, 0x00, 0x00, 0x90 };
		REL::safe_write<uint8_t>(ptr_ADS_DistanceCheck.address(), std::span{ bytes });
		for (auto it = GameSettingCollection::GetSingleton()->settings.begin(); it != GameSettingCollection::GetSingleton()->settings.end(); ++it) {
			if (it->first == "fPlayerCoverPeekTime") {
				it->second->SetFloat(0.0f);
				_MESSAGE("%s changed to %f", it->first.c_str(), it->second->_value);
			}
		}
		vanillaPeekPatched = true;
	}
}

void PreparePlayerSkeleton()
{
	NiAVObject* fpNode = p->Get3D(true);
	if (fpNode) {
		//_MESSAGE("First person skeleton found.");

		NiNode* chest = (NiNode*)fpNode->GetObjectByName("Chest");
		if (chest) {
			InsertBone(fpNode, chest, "ChestInserted1st");
		}

		NiNode* com = (NiNode*)fpNode->GetObjectByName("COM");
		if (com) {
			InsertBone(fpNode, com, "COMInserted1st");
		}

		NiNode* camera = (NiNode*)fpNode->GetObjectByName("Camera");
		if (camera) {
			InsertBone(fpNode, camera, "CameraInserted1st");
		}
	}
	NiAVObject* tpNode = p->Get3D(false);
	if (tpNode) {
		//_MESSAGE("Third person skeleton found.");

		NiNode* com = (NiNode*)tpNode->GetObjectByName("COM");
		if (com) {
			InsertBone(tpNode, com, "COMInserted");
		}

		NiNode* camera = (NiNode*)tpNode->GetObjectByName("Camera");
		if (camera) {
			InsertBone(tpNode, camera, "CameraInserted");
		}

		NiNode* spine1 = (NiNode*)tpNode->GetObjectByName("SPINE1");
		if (spine1) {
			InsertBone(tpNode, spine1, "Spine1Inserted");
		}

		NiNode* spine2 = (NiNode*)tpNode->GetObjectByName("SPINE2");
		if (spine2) {
			InsertBone(tpNode, spine2, "Spine2Inserted");
		}

		NiNode* chest = (NiNode*)tpNode->GetObjectByName("Chest");
		if (chest) {
			InsertBone(tpNode, chest, "ChestInserted");
		}

		NiNode* pelvis = (NiNode*)tpNode->GetObjectByName("Pelvis");
		if (pelvis) {
			InsertBone(tpNode, pelvis, "PelvisInserted");
		}

		NiNode* llegCalf = (NiNode*)tpNode->GetObjectByName("LLeg_Calf");
		if (llegCalf) {
			InsertBone(tpNode, llegCalf, "LLeg_CalfInserted");
		}

		NiNode* rlegCalf = (NiNode*)tpNode->GetObjectByName("RLeg_Calf");
		if (rlegCalf) {
			InsertBone(tpNode, rlegCalf, "RLeg_CalfInserted");
		}
	}
}

void ResizeCollisionPolytopeShapeX(hknpShape* shape, vector<hkVector4f>& cache, float ratio, float bumperHalf, float bumperDiff)
{
	uint16_t vertexSize = *(uint16_t*)((uintptr_t)shape + 0x30);
	uint16_t vertexOffset = *(uint16_t*)((uintptr_t)shape + 0x32);
	if (cache.size() == 0) {
		for (int i = 0; i < vertexSize; ++i) {
			cache.push_back(*(hkVector4f*)((uintptr_t)shape + 0x30 + vertexOffset + 0x10 * i));
		}
	}
	for (int i = 0; i < cache.size(); ++i) {
		((hkVector4f*)((uintptr_t)shape + 0x30 + vertexOffset + 0x10 * i))->x = cache[i].x + cache[i].x * ratio + bumperDiff;
	}
}

void ResizeCollisionShapeX(CompoundShapeData* data, vector<hkVector4f>& cache, float ratio, float bumperHalf, float bumperDiff)
{
	ResizeCollisionPolytopeShapeX(data->shape, cache, ratio, bumperHalf, bumperDiff);
}

void SetLeanState(int state)
{
	lastLeanState = leanState;
	leanState = state;
}

#pragma endregion

#pragma region Events

class LeanLookHandler
{
public:
	typedef void (LeanLookHandler::*FnOnMouseMoveEvent)(MouseMoveEvent* evn);
	typedef void (LeanLookHandler::*FnOnThumbstickEvent)(ThumbstickEvent* evn);

	void RotateXY(float x, float y, float rad, std::vector<float>& v)
	{
		v.push_back(cos(rad) * x - sin(rad) * y);
		v.push_back(sin(rad) * x + cos(rad) * y);
	}

	void HookedMouseMoveEvent(MouseMoveEvent* evn)
	{
		if (pcam->currentState == pcam->cameraStates[CameraState::kFirstPerson] && !leanR6Style) {
			std::vector<float> input;
			RotateXY((float)evn->mouseInputX, (float)evn->mouseInputY, -rotZ * toRad, input);
			leanCumulativeMouseX += input[0];
			leanCumulativeMouseY += input[1];
			evn->mouseInputX = (int32_t)leanCumulativeMouseX;
			evn->mouseInputY = (int32_t)leanCumulativeMouseY;
			leanCumulativeMouseX -= (int32_t)leanCumulativeMouseX;
			leanCumulativeMouseY -= (int32_t)leanCumulativeMouseY;
		}
		(this->*mouseEvn)(evn);
	}

	void HookedThumbstickEvent(ThumbstickEvent* evn)
	{
		if (evn->idCode == ThumbstickEvent::kRight && pcam->currentState == pcam->cameraStates[CameraState::kFirstPerson] && !leanR6Style) {
			std::vector<float> input;
			RotateXY(evn->xValue, -evn->yValue, -rotZ * toRad, input);
			evn->xValue = input[0];
			evn->yValue = -input[1];
		}
		(this->*thumbstickEvn)(evn);
	}

	void HookEvents()
	{
		uintptr_t vtable = *(uintptr_t*)this;
		mouseEvn = SafeWrite64Function(vtable + 0x30, &LeanLookHandler::HookedMouseMoveEvent);
		thumbstickEvn = SafeWrite64Function(vtable + 0x20, &LeanLookHandler::HookedThumbstickEvent);
	}

protected:
	static FnOnMouseMoveEvent mouseEvn;
	static FnOnThumbstickEvent thumbstickEvn;
};
LeanLookHandler::FnOnMouseMoveEvent LeanLookHandler::mouseEvn;
LeanLookHandler::FnOnThumbstickEvent LeanLookHandler::thumbstickEvn;

void HookedUpdate()
{
	float curTime = *F4::ptr_engineTime;
	if (iniDevMode && curTime - lastiniUpdate > 5.0f) {
		LoadConfigs();
		lastiniUpdate = curTime;
	}
	Actor* a = p;
	NiAVObject* node = a->Get3D();
	NiAVObject* tpNode = a->Get3D(false);
	if (node && tpNode && (a->moreFlags & 0x20) == 0 && a->GetObjectReference()->formID == 0x7) {
		NiNode* head = (NiNode*)tpNode->GetObjectByName("HEAD");
		NiNode* pelvis = (NiNode*)tpNode->GetObjectByName("Pelvis");
		NiNode* root = (NiNode*)tpNode->GetObjectByName("Root");
		if (node != lastRoot) {
			bbx = (BSBound*)tpNode->GetExtraData("BBX");
			//_MESSAGE("tpNode %llx", tpNode);
			lastRoot = node->IsNode();
			PreparePlayerSkeleton();
		}
		float deltaTime = min(curTime - lastRun, 5.0f);
		float timeMult = deltaTime * 60.0f;
		bool isFP = IsFirstPerson();
		float pcScale = GetActorScale(p);
		float heightRatio = 1;
		float transDist = 0;
		NiPoint3 pos, dir, right;
		a->GetEyeVector(pos, dir, true);
		NiPoint3 heading = Normalize(NiPoint3(dir.x, dir.y, 0));
		if (root) {
			right = ToRightVector(root->world.rotate);
		}

		bhkCharacterController* con = nullptr;
		if (a->currentProcess) {
			con = a->currentProcess->middleHigh->charController.get();
		}

#pragma region Inertia
		float tempRotDivX = rotDivX;
		float tempRotDivY = rotDivY;
		float conditionalMultiplier = 1.0f;
		if (a->GetActorValue(*leftAttackCondition) == 0) {
			conditionalMultiplier -= 0.2f;
		}
		if (a->GetActorValue(*rightAttackCondition) == 0) {
			conditionalMultiplier -= 0.2f;
		}
		if (IsInADS(a)) {
			conditionalMultiplier *= rotADSConditionMult;
			tempRotDivX = rotDivXADS;
			tempRotDivY = rotDivYADS;
		}
		float step = rotReturnStep * conditionalMultiplier * timeMult;
		float retDiv = max(pow(rotReturnDiv * conditionalMultiplier, timeMult), rotReturnDivMin);
		float followDiv = max(pow(3.0f, timeMult), 1.0f);
		float eulerX, eulerY, eulerZ;
		(pcam->cameraRoot->world.rotate * Transpose(lastCamRot)).ToEulerAnglesXYZ(eulerX, eulerY, eulerZ);
		targetRotZ -= eulerZ / toRad / tempRotDivY * 5.f;
		targetRotX -= eulerX / toRad / tempRotDivX * 5.f;
		rotX += (targetRotX - rotX) / followDiv;
		rotX = max(min(rotX, rotLimitX), -rotLimitX);
		rotY += (targetRotZ - rotY) / followDiv;
		rotY = max(min(rotY, rotLimitY), -rotLimitY);
		targetRotZ /= retDiv;
		targetRotX /= retDiv;
		lastCamRot = pcam->cameraRoot->world.rotate;
		//_MESSAGE("retDiv %f rotX %f rotY %f", retDiv, rotX, rotY);
		if (abs(rotX) * (abs(rotX) - step) <= 0) {
			rotX = 0;
		} else {
			if (rotX > 0) {
				rotX -= step;
			} else {
				rotX += step;
			}
		}
		if (abs(rotY) * (abs(rotY) - step) <= 0) {
			rotY = 0;
		} else {
			if (rotY > 0) {
				rotY -= step;
			} else {
				rotY += step;
			}
		}
		if (IsInADS(a) && rotDisableInADS) {
			rotX = 0;
			rotY = 0;
			targetRotZ = 0;
			targetRotX = 0;
		}
#pragma endregion

		if (leanADSOnly && !IsInADS(a)) {
			SetLeanState(0);
		}

		if (leanState != 0) {
			UI* ui = UI::GetSingleton();
			if (ui->GetMenuOpen("WorkshopMenu") || ui->GetMenuOpen("DialogueMenu") || ui->GetMenuOpen("LooksMenu") || a->interactingState != INTERACTING_STATE::kNotInteracting || (a->moveMode & 0x100) == 0x100 || (*(uint32_t*)((uintptr_t)a + 0x130) & 0x1E00000) == 0xE00000) {
				SetLeanState(0);
			}
		}

		if (!isFP && head) {
			pos = head->world.translate;
		}
		if (leanState != 0) {
			F4::bhkPickData camLeanCheckPick = F4::bhkPickData();
			GetPickData(pos, pos - right * 30.f * (float)leanState, a, nullptr, camLeanCheckPick);
			F4::bhkPickData camLeanRevertPick = F4::bhkPickData();
			GetPickData(pos, pos - right * 22.5f * (float)leanState, a, nullptr, camLeanRevertPick);
			if (!camLeanCheckPick.HasHit()) {
				if (leanState == 1) {
					leanTime = min(leanTime + deltaTime, leanTimeCost);
				} else if (leanState == -1) {
					leanTime = max(leanTime - deltaTime, -leanTimeCost);
				}
			} else {
				if (collisionDevMode) {
					_MESSAGE("Camera colliding! No Lean");
				}
				if (camLeanRevertPick.HasHit()) {
					if (leanState == 1) {
						leanTime = max(leanTime - deltaTime, 0);
					} else if (leanState == -1) {
						leanTime = min(leanTime + deltaTime, 0);
					}
					if (collisionDevMode) {
						_MESSAGE("Camera hard collide! Reverting Lean");
					}
				}
			}
		} else {
			if (abs(leanTime) <= deltaTime) {
				leanTime = 0;
			} else {
				leanTime = max(min(leanTime - Sign(leanTime) * deltaTime, leanTimeCost), -leanTimeCost);
			}
			leanCumulativeMouseX = 0.f;
			leanCumulativeMouseY = 0.f;
		}
		float deltaLeanWeight = leanTime / leanTimeCost - leanWeight;
		if ((leanWeight + deltaLeanWeight > 0 && leanWeight <= 0) || (leanWeight + deltaLeanWeight < 0 && leanWeight >= 0)) {
			leanDistMoved = 0;
		}
		leanWeight += deltaLeanWeight;

		rotZ = isFP ? leanMax * leanWeight : leanMax3rd * leanWeight;
		transZ = isFP ? leanMax * leanWeight / 3.0f : leanMax3rd * leanWeight / 3.0f;
		float rotXRadByThree = rotX * toRad / 3.0f;
		float rotYRadByTwo = rotY * toRad / 2.0f;
		float rotZRadByThree = rotZ * toRad / 3.0f;
		float transZRad = transZ * toRad;
		float transZByTwo = transZ / 2.0f;
		float transX = 0.f;

		if (con) {
			if (head && pelvis) {
				float height = max(max(head->world.translate.z, pelvis->world.translate.z) + heightBuffer - a->data.location.z, 10.0f);
				//_MESSAGE("Current height %f", height);
				if (bbx) {
					float optimalHeight = bbx->extents.z * 2.0f;
					heightRatio = height / optimalHeight;
					transDist = isFP ? optimalHeight * sin(leanMax * toRad) : bbx->extents.z * sin(leanMax3rd * toRad);
					transDist *= pcScale;
					transX = isFP ? transDist * leanWeight * heightRatio : transDist * leanWeight;
					//_MESSAGE("Optimal height %f", optimalHeight);
					if (deltaLeanWeight != 0) {
						if (collisionDevMode) {
							_MESSAGE("transDist %f", transDist);
						}
						hknpShape* colShape = con->shapes[1]._ptr;
						if (colShape) {
							float ratio = 1.f + abs(transX) / (bbx->extents.x * pcScale) / 2.f;
							MultiCompoundShape* multiShape = *(MultiCompoundShape**)((uintptr_t)colShape + 0x60);
							if (multiShape) {
								int32_t numShape = *(int32_t*)((uintptr_t)colShape + 0x68);
								for (int i = 0; i < numShape; ++i) {
									multiShape->data[i].translate.x = (ratio - 1.f) * Sign(transX) / 2.f;
									multiShape->data[i].scale.x = ratio;
								}
							}
							hknpDynamicCompoundShapeData* shapeData = *(hknpDynamicCompoundShapeData**)((uintptr_t)colShape + 0xC0);
							if (shapeData) {
								if (cachedShape[4].size() == 0) {
									for (int i = 0; i < 8; ++i) {
										cachedShape[4].push_back(shapeData->bbv->vertex[i]);
									}
								}
								for (int i = 0; i < cachedShape[4].size(); ++i) {
									shapeData->bbv->vertex[i].x = cachedShape[4][i].x * ratio;
								}
							}
						}
						uintptr_t charProxy = *(uintptr_t*)((uintptr_t)con + 0x470);
						if (charProxy) {
							hkTransform* charProxyTransform = (hkTransform*)(charProxy + 0x40);
							hkVector4f* charProxyVel = (hkVector4f*)(charProxy + 0xA0);
							hkVector4f& charProxyLastDisplacement = *(hkVector4f*)(charProxy + 0xB0);
							//_MESSAGE("Length %f", (charProxyLastDisplacement - charProxyVel * con->stepInfo.deltaTime.storage).Length());
							//float diff = DotProduct(charProxyVel - con->velocityMod, right * -deltaLeanWeight);
							float deltaDist = transDist * deltaLeanWeight / HAVOKTOFO4;
							F4::bhkPickData pick = F4::bhkPickData();
							GetPickData(a->data.location + NiPoint3(0, 0, 25.f), a->data.location + NiPoint3(0, 0, 25.f) - right * 1000.f * Sign(deltaLeanWeight), a, nullptr, pick);
							bool colDetected = false;
							hkVector4f displacement = right * -deltaDist;
							if (pick.HasHit()) {
								NiPoint3 colPos = NiPoint3(*(float*)((uintptr_t)&pick + 0x60),
													  *(float*)((uintptr_t)&pick + 0x64),
													  *(float*)((uintptr_t)&pick + 0x68)) /
								                  *ptr_fBS2HkScale;
								float colDist = Length(colPos - a->data.location + NiPoint3(0, 0, 25.f));
								if (colDist < transDist) {
									colDetected = true;
								}
								if (collisionDevMode) {
									_MESSAGE("Collision Detected colDist %f", colDist);
								}
							}
							if (!colDetected) {
								//a->ApplyMovementDelta(deltaTime, displacementA, NiPoint3());
								//((ActorEx*)a)->Move(deltaTime, displacementA, true);
								charProxyTransform->m_translation.v.x += displacement.x;
								charProxyTransform->m_translation.v.y += displacement.y;
								charProxyTransform->m_translation.v.z += displacement.z;
							}
						}
					}

					if (dynamicHeight && !isLoading) {
						if (abs(height - lastHeight) > heightDiffThreshold) {
							for (int sn = 0; sn < 2; ++sn) {
								hknpShape* bumper = con->shapes[sn]._ptr;
								if (bumper) {
									CompoundShapeData* data = *(CompoundShapeData**)((uintptr_t)bumper + 0x60);
									if (data) {
										data->translate.z = heightRatio - 0.98f;
										data->scale.z = heightRatio;
									}
								}
							}
							//_MESSAGE("Changed height");
							lastHeight = height;
						}
					}
				}
			}
		}

		NiPoint3 colTransX = NiPoint3(transX, 0, 0);
		NiNode* comInserted = (NiNode*)tpNode->GetObjectByName("COMInserted");
		if (comInserted) {
			comInserted->local.translate = colTransX;
		}

		NiNode* cameraInserted = (NiNode*)tpNode->GetObjectByName("CameraInserted");
		if (cameraInserted) {
			cameraInserted->local.translate = colTransX;
		}

		NiNode* chestInserted = (NiNode*)tpNode->GetObjectByName("ChestInserted");
		if (chestInserted) {
			NiMatrix3 rot = chestInserted->parent->world.rotate * GetRotationMatrix33(heading, rotZRadByThree) * Transpose(chestInserted->parent->world.rotate);
			chestInserted->local.rotate = rot;
		}

		NiNode* spine2Inserted = (NiNode*)tpNode->GetObjectByName("Spine2Inserted");
		if (spine2Inserted) {
			spine2Inserted->local.rotate = spine2Inserted->parent->world.rotate * GetRotationMatrix33(heading, rotZRadByThree) * Transpose(spine2Inserted->parent->world.rotate);
			//_MESSAGE("spine1Inserted");
		}

		NiNode* spine1Inserted = (NiNode*)tpNode->GetObjectByName("Spine1Inserted");
		if (spine1Inserted) {
			spine1Inserted->local.rotate = spine1Inserted->parent->world.rotate * GetRotationMatrix33(heading, rotZRadByThree) * Transpose(spine1Inserted->parent->world.rotate);
			spine1Inserted->local.translate.z = transZ;
			//_MESSAGE("spine1Inserted");
		}

		NiNode* pelvisInserted = (NiNode*)tpNode->GetObjectByName("PelvisInserted");
		if (pelvisInserted) {
			NiMatrix3 rot = GetRotationMatrix33(rotZRadByThree, 0, 0);
			pelvisInserted->local.rotate = rot;
			pelvisInserted->local.translate.z = transZ;
			//_MESSAGE("pelvisInserted");
		}

		NiNode* llegCalfInserted = (NiNode*)tpNode->GetObjectByName("LLeg_CalfInserted");
		if (llegCalfInserted) {
			NiMatrix3 rot = GetRotationMatrix33(-rotZRadByThree, 0, 0);
			llegCalfInserted->local.rotate = rot;
			llegCalfInserted->local.translate.z = transZByTwo;
			//_MESSAGE("llegFootInserted");
		}

		NiNode* rlegCalfInserted = (NiNode*)tpNode->GetObjectByName("RLeg_CalfInserted");
		if (rlegCalfInserted) {
			NiMatrix3 rot = GetRotationMatrix33(-rotZRadByThree, 0, 0);
			rlegCalfInserted->local.rotate = rot;
			rlegCalfInserted->local.translate.z = transZByTwo;
			//_MESSAGE("rlegFootInserted");
		}

		if (isFP) {
			NiNode* cameraInserted1st = (NiNode*)node->GetObjectByName("CameraInserted1st");
			NiNode* camera = (NiNode*)node->GetObjectByName("Camera");
			if (camera && cameraInserted1st) {
				if (!leanR6Style) {
					NiMatrix3 rot = GetRotationMatrix33(rotZ * toRad, 0, 0);
					cameraInserted1st->local.translate = colTransX;
					cameraInserted1st->local.rotate = rot;
				} else {
					NiMatrix3 rot = GetRotationMatrix33(-rotZ * toRad, 0, 0);
					NiPoint3 zoomData = NiPoint3();
					if (p->currentProcess && p->currentProcess->middleHigh) {
						BSTArray<EquippedItem> equipped = p->currentProcess->middleHigh->equippedItems;
						if (equipped.size() != 0 && equipped[0].item.instanceData) {
							TESObjectWEAP::InstanceData* instance = (TESObjectWEAP::InstanceData*)equipped[0].item.instanceData.get();
							if (instance->type == 9 && instance->zoomData) {
								zoomData = instance->zoomData->zoomData.cameraOffset;
							}
						}
					}
					NiPoint3 camPos = camera->local.translate + zoomData;
					cameraInserted1st->local.translate = rot * camPos - camPos + colTransX;
					cameraInserted1st->local.rotate.MakeIdentity();
				}
			}

			//float rayDist = 70.0f * abs(sin(leanMax * toRad));
			//CastRay(a, rayOrigin, rayDir, rayDist);
			colTransX = NiPoint3(transDist * leanWeight * heightRatio, 0, 0);
			NiNode* chestInserted1st = (NiNode*)node->GetObjectByName("ChestInserted1st");
			if (camera && chestInserted1st) {
				NiMatrix3 rot = chestInserted1st->parent->world.rotate * GetRotationMatrix33(ToRightVector(camera->world.rotate), rotX * toRad) * GetRotationMatrix33(ToUpVector(camera->world.rotate), rotY * toRad) * Transpose(chestInserted1st->parent->world.rotate);  //GetRotationMatrix33(0, rotY * toRad, -rotX * toRad);
				chestInserted1st->local.rotate = rot;
			}

			NiNode* comInserted1st = (NiNode*)node->GetObjectByName("COMInserted1st");
			if (comInserted1st) {
				NiMatrix3 rot = GetRotationMatrix33(rotZ * toRad, 0, 0);
				comInserted1st->local.rotate = rot;
				comInserted1st->local.translate = colTransX;
				//_MESSAGE("comInserted");
			}
		}
	} else {
		SetLeanState(0);
	}
	lastRun = curTime;
	/*typedef void (*FnUpdateSceneGraph)(PlayerCharacter*);
	FnUpdateSceneGraph fn = (FnUpdateSceneGraph)UpdateSceneGraphOrig;
	if (fn)
		fn(a);*/
	typedef void (*FnUpdate)();
	FnUpdate fn = (FnUpdate)PCUpdateMainThreadOrig;
	if (fn)
		(*fn)();
}

class InputEventReceiverOverride : public BSInputEventReceiver
{
public:
	typedef void (InputEventReceiverOverride::*FnPerformInputProcessing)(InputEvent* a_queueHead);

	void ProcessButtonEvent(const ButtonEvent* evn)
	{
		uint32_t id = evn->idCode;
		if (evn->device == INPUT_DEVICE::kMouse)
			id += 256;

		if (buttonDevMode) {
			_MESSAGE("Button event fired id %d held %f", id, evn->heldDownSecs);
		}

		if (!leanADSOnly || (leanADSOnly && IsInADS(p))) {
			if (toggleLean) {
				if (evn->value && evn->heldDownSecs == 0) {
					if ((leanState == 1 && id == leanLeft) || (leanState == -1 && id == leanRight)) {
						SetLeanState(0);
					} else {
						if (id == leanLeft) {
							SetLeanState(1);
						} else if (id == leanRight) {
							SetLeanState(-1);
						}
					}
				}
			} else {
				if (evn->value && evn->heldDownSecs == 0) {
					if (id == leanRight) {
						SetLeanState(-1);
					} else if (id == leanLeft) {
						SetLeanState(1);
					}
				} else if (!evn->value) {
					if ((id == leanLeft && leanState == 1) || (id == leanRight && leanState == -1)) {
						SetLeanState(0);
					}
				}
			}
		}
	}

	void HookedPerformInputProcessing(InputEvent* a_queueHead)
	{
		if (!UI::GetSingleton()->menuMode && !disableLean) {
			InputEvent* evn = a_queueHead;
			while (evn) {
				if (evn->eventType == INPUT_EVENT_TYPE::kButton) {
					ProcessButtonEvent((ButtonEvent*)evn);
				}
				evn = evn->next;
			}
		}
		FnPerformInputProcessing fn = fnHash.at(*(uint64_t*)this);
		if (fn) {
			(this->*fn)(a_queueHead);
		}
	}

	void HookSink()
	{
		uint64_t vtable = *(uint64_t*)this;
		auto it = fnHash.find(vtable);
		if (it == fnHash.end()) {
			FnPerformInputProcessing fn = SafeWrite64Function(vtable, &InputEventReceiverOverride::HookedPerformInputProcessing);
			fnHash.insert(std::pair<uint64_t, FnPerformInputProcessing>(vtable, fn));
		}
	}

	void UnHookSink()
	{
		uint64_t vtable = *(uint64_t*)this;
		auto it = fnHash.find(vtable);
		if (it == fnHash.end())
			return;
		SafeWrite64Function(vtable, it->second);
		fnHash.erase(it);
	}

protected:
	static std::unordered_map<uint64_t, FnPerformInputProcessing> fnHash;
};
std::unordered_map<uint64_t, InputEventReceiverOverride::FnPerformInputProcessing> InputEventReceiverOverride::fnHash;

class ObjectLoadWatcher : public BSTEventSink<TESObjectLoadedEvent>
{
public:
	virtual BSEventNotifyControl ProcessEvent(const TESObjectLoadedEvent& evn, BSTEventSource<TESObjectLoadedEvent>* a_source)
	{
		if (!evn.loaded) {
			if (evn.formId == 0x14) {
				_MESSAGE("Player unloaded");
			}
			return BSEventNotifyControl::kContinue;
		} else {
			if (evn.formId == 0x14) {
				_MESSAGE("Player loaded");
				playerLastLoaded = *F4::ptr_engineTime;
				//PreparePlayerSkeleton();
			}
		}
		return BSEventNotifyControl::kContinue;
	}
	F4_HEAP_REDEFINE_NEW(ObjectLoadWatcher);
};

class MenuWatcher : public BSTEventSink<MenuOpenCloseEvent>
{
	virtual BSEventNotifyControl ProcessEvent(const MenuOpenCloseEvent& evn, BSTEventSource<MenuOpenCloseEvent>* src) override
	{
		if (evn.menuName == BSFixedString("LoadingMenu")) {
			if (evn.opening) {
				isLoading = true;
				leanState = 0;
			} else {
				isLoading = false;
			}
		}
		return BSEventNotifyControl::kContinue;
	}
};

#pragma endregion

#pragma region Initializers

void InitializePlugin()
{
	p = PlayerCharacter::GetSingleton();
	pcam = PlayerCamera::GetSingleton();
	((InputEventReceiverOverride*)((uint64_t)pcam + 0x38))->HookSink();
	_MESSAGE("PlayerCharacter %llx PlayerCamera %llx", p, pcam);
	pc = PlayerControls::GetSingleton();
	((LeanLookHandler*)pc->lookHandler)->HookEvents();
	leftAttackCondition = (ActorValueInfo*)TESForm::GetFormByID(0x00036E);
	rightAttackCondition = (ActorValueInfo*)TESForm::GetFormByID(0x00036F);
	colCheckProj = (BGSProjectile*)TESForm::GetFormByID(0xE942D);
	MQ102 = (TESQuest*)TESForm::GetFormByID(0x1CC2A);
	_MESSAGE("Hooked to event");
	ObjectLoadWatcher* olw = new ObjectLoadWatcher();
	ObjectLoadedEventSource::GetSingleton()->RegisterSink(olw);
	MenuWatcher* mw = new MenuWatcher();
	UI::GetSingleton()->GetEventSource<MenuOpenCloseEvent>()->RegisterSink(mw);
}

#pragma endregion

extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Query(const F4SE::QueryInterface* a_f4se, F4SE::PluginInfo* a_info)
{
#ifndef NDEBUG
	auto sink = std::make_shared<spdlog::sinks::msvc_sink_mt>();
#else
	auto path = logger::log_directory();
	if (!path) {
		return false;
	}

	*path /= fmt::format(FMT_STRING("{}.log"), Version::PROJECT);
	auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
#endif

	auto log = std::make_shared<spdlog::logger>("global log"s, std::move(sink));

#ifndef NDEBUG
	log->set_level(spdlog::level::trace);
#else
	log->set_level(spdlog::level::info);
	log->flush_on(spdlog::level::warn);
#endif

	spdlog::set_default_logger(std::move(log));
	spdlog::set_pattern("%g(%#): [%^%l%$] %v"s);

	logger::info(FMT_STRING("{} v{}"), Version::PROJECT, Version::NAME);

	a_info->infoVersion = F4SE::PluginInfo::kVersion;
	a_info->name = Version::PROJECT.data();
	a_info->version = Version::MAJOR;

	if (a_f4se->IsEditor()) {
		logger::critical("loaded in editor"sv);
		return false;
	}

	const auto ver = a_f4se->RuntimeVersion();
	if (ver < F4SE::RUNTIME_1_10_162) {
		logger::critical(FMT_STRING("unsupported runtime v{}"), ver.string());
		return false;
	}

	F4SE::AllocTrampoline(8 * 8);

	return true;
}

extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Load(const F4SE::LoadInterface* a_f4se)
{
	F4SE::Init(a_f4se);

	F4SE::Trampoline& trampoline = F4SE::GetTrampoline();
	PCUpdateMainThreadOrig = trampoline.write_call<5>(ptr_PCUpdateMainThread.address(), &HookedUpdate);
	//UpdateSceneGraphOrig = trampoline.write_call<5>(ptr_UpdateSceneGraph.address(), &HookedUpdateSceneGraph);

	const F4SE::MessagingInterface* message = F4SE::GetMessagingInterface();
	message->RegisterListener([](F4SE::MessagingInterface::Message* msg) -> void {
		if (msg->type == F4SE::MessagingInterface::kGameDataReady) {
			InitializePlugin();
			LoadConfigs();
		} else if (msg->type == F4SE::MessagingInterface::kPostLoadGame) {
			LoadConfigs();
		} else if (msg->type == F4SE::MessagingInterface::kNewGame) {
			LoadConfigs();
		}
	});

	return true;
}
