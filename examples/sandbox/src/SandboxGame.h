#pragma once

#include <memory>
#include <string>
#include <vector>

#include <raylib.h>

#include <ai/Flocking.h>
#include <cyclone/cyclone.h>
#include <engine/FlyCamera.h>
#include <engine/Game.h>
#include <engine/GraphicsBufferManager.h>
#include <engine/Lighting.h>
#include <engine/Sky.h>
#include <engine/Terrain.h>
#include <engine/Unit.h>

#include "InputTranslator.h"

#ifdef TEMPLATE_HAS_SOCKETS
#include <net/LanDiscovery.h>
#include <net/Packet.h>
#include <net/TcpSocket.h>
#include <net/UdpSocket.h>
#endif

#ifdef TEMPLATE_HAS_RELAY
#include <net/RelayClient.h>
#endif

#ifdef TEMPLATE_HAS_GPU_COMPUTE
#include <gpu/BallSimulation.h>
#endif

// A body in the physics playground; the world keeps pointers, so these live on the heap
enum class PlaygroundShape
{
    BOX,
    BALL,
    CONVEX
};

struct PlaygroundBody
{
    cyclone::RigidBody body;
    std::unique_ptr<cyclone::Collider> pCollider;
    PlaygroundShape shape = PlaygroundShape::BOX;
    int convexModel = -1;
    Color color = WHITE;
    int fractureLevelsLeft = 0; // Breaks on a hard hit while above 0
    cyclone::Vector3 velocityBeforeStep;

    // The pose before the last physics step, for drawing between steps
    cyclone::Vector3 previousPosition;
    cyclone::Quaternion previousOrientation;
};

// Everything game.yaml can change
struct SandboxSettings
{
    engine::GameConfig game{ "CppTemplate Sandbox" };
    float restitution = 0.6f;
    std::string relayUrl = "ws://localhost:9000";
};

// The starting scene: replace its contents with your project
class SandboxGame : public engine::Game
{
public:
    explicit SandboxGame(const SandboxSettings& settings);

private:
    void onInit() override;
    void onShutdown() override;
    void onFixedUpdate(double dt) override;
    void onUpdate(double frameDt, double alpha) override;
    void onRenderOffscreen() override;
    void onDraw3D() override;
    void onDraw2D() override;
    void onGui() override;
    void handleEvent(const core::Event& theEvent) override;

    void resetScene();
    void spawnFlock();
    void createUnitSprites();
    void createSounds();
    void handleUnitEvent(const core::Event& theEvent);
    void drawScene();
    void buildPlayground();
    PlaygroundBody* addPlaygroundBox(Vector3 position, Vector3 halfSize, float mass);
    PlaygroundBody* addPlaygroundBall(Vector3 position, float radius, float mass);
    PlaygroundBody* addPlaygroundConvex(int shapeIndex, Vector3 position, float mass);
    PlaygroundBody* addPlaygroundBody(std::unique_ptr<PlaygroundBody> pBody);
    void removePlaygroundBody(PlaygroundBody* pBody);
    void stepPlayground(double dt);
    void fractureHitBodies();
    Matrix interpolatedTransform(const PlaygroundBody& body) const;
    void throwBall();
    void dropBallOnGlass();
    void drawPlayground();
    void drawPlaygroundContacts();
    void drawPlaygroundGui();
    Vector3 getSunDirection() const;

    std::unique_ptr<InputTranslator> mpTranslator;
    engine::FlyCamera mFlyCamera;

    cyclone::RigidBody mBody;
    cyclone::Vector3 mPreviousPosition;
    Vector3 mRenderPosition{};
    Model mCubeModel{};
    float mRestitution = 0.6f;
    bool mPaused = false;
    int mBounces = 0;

    engine::Lighting mLighting;
    engine::Sky mSky;
    engine::Terrain mTerrain;
    engine::TerrainSettings mTerrainSettings;
    bool mShadows = true;
    float mTimeOfDay = 0.15f; // 0 sunrise, 0.25 noon, 0.5 sunset, 0.75 midnight

    cyclone::World mPhysics{ 4096 };
    cyclone::BoxCollider mPlatform;
    cyclone::PlaneCollider mCatchFloor;
    cyclone::Explosion mExplosion;
    std::vector<std::unique_ptr<PlaygroundBody>> mPlayBodies;
    std::vector<cyclone::ConvexCollider> mConvexShapes;
    std::vector<Model> mConvexModels;
    float mRenderAlpha = 1.0f;
    int mBroadPhase = 1;
    int mResolver = 0;
    std::vector<std::unique_ptr<cyclone::Joint>> mJoints;
    Vector3 mPlaygroundCenter{};
    Model mUnitCube{};
    Model mUnitSphere{};
    int mThrownBalls = 0;
    bool mShowContacts = false;

    ai::Flock mFlock;
    Model mBoidModel{};
    bool mShowFlock = true;

    engine::GraphicsBufferManager mBuffers;
    engine::UnitManager mUnits;
    engine::Animation mUnitSpin;
    engine::Animation mUnitPulse;
    bool mUnitsFrozen = false;

#ifdef TEMPLATE_HAS_SOCKETS
    void initLoopback();
    void pollLoopback();
    void drawSocketGui();

    net::UdpSocket mServer;
    net::UdpSocket mClient;
    net::PacketDispatcher mServerDispatcher;
    net::PacketDispatcher mClientDispatcher;
    std::uint32_t mPingsSent = 0;
    std::uint32_t mPongsReceived = 0;
    double mLastRoundTripMS = 0.0;

    net::TcpListener mTcpListener;
    std::unique_ptr<net::TcpConnection> mpTcpServerSide;
    net::TcpConnection mTcpClient;
    char mChatInput[128] = "";
    std::vector<std::string> mChatLog;

    net::LanDiscovery mDiscovery;
    bool mHostOnLan = false;
#endif

#ifdef TEMPLATE_HAS_GPU_COMPUTE
    void initBalls();
    void spawnBalls();
    void stepBalls(double dt);
    void updateBalls();
    void drawBalls();
    void drawBallGui();
    void unloadBalls();
    void setBallsOnGpu(bool useGpu);
    bool ballsOnGpu() const;
    const std::vector<gpu::Float4>& getBallSpheres() const;

    gpu::Context mGpu;
    gpu::CpuBallSimulation mCpuBalls;
    gpu::GpuBallSimulation mGpuBalls;
    gpu::BallSettings mBallSettings;
    bool mWantGpu = true;
    bool mGpuInitDone = false;
    int mBallCount = 8000;
    double mCpuStepMS = 0.0;
    Mesh mBallMesh{};
    Material mBallMaterial{};
    int mBallSunLocation = -1;
    int mBallRangeLocation = -1;
    std::vector<Matrix> mBallTransforms;
#endif

#ifdef TEMPLATE_HAS_RELAY
    void initRelay();
    void drawRelayGui();

    net::RelayClient mRelay;
    char mRelayUrl[128] = "ws://localhost:9000";
    char mJoinCode[16] = "";
    std::string mRelayStatus = "Not connected";
    std::string mLobbyCode;
    int mPeerResets = 0;
#endif
};
