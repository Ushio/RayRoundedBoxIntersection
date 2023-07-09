#include "pr.hpp"

#include <iostream>
#include <memory>
#include <ppl.h>
#include <set>

template <class T>
inline T ss_max( T x, T y )
{
	return ( x < y ) ? y : x;
}
template <class T>
inline T ss_min( T x, T y )
{
	return ( y < x ) ? y : x;
}
template <class T>
inline T ss_abs( T x )
{
	return x >= T( 0 ) ? x : -x;
}

inline float maxElement( float a, float b, float c )
{
	return ss_max( ss_max( a, b ), c );
}
inline float minElement( float a, float b, float c )
{
	return ss_min( ss_min( a, b ), c );
}
inline float maxElement( glm::vec3 x )
{
	return maxElement( x.x, x.y, x.z );
}
inline float minElement( glm::vec3 x )
{
	return minElement( x.x, x.y, x.z );
}

// lower t is set to tout which can be negative.
bool hasIntersectionBox( glm::vec3 ro, glm::vec3 one_over_rd, glm::vec3 lower, glm::vec3 upper, float* tout )
{
	glm::vec3 t0 = ( lower - ro ) * one_over_rd;
	glm::vec3 t1 = ( upper - ro ) * one_over_rd;
	glm::vec3 tL = glm::min( t0, t1 );
	glm::vec3 tU = glm::max( t0, t1 );

	float S_lmax = maxElement( tL.x, tL.y, tL.z );
	float S_umin = minElement( tU.x, tU.y, tU.z );
	if( 0.0f < S_umin && S_lmax < S_umin )
	{
		*tout = S_lmax;
		return true;
	}
	return false;
}

// WideH must be larger than equal to R
// backface culled
bool intersectRoundedbox( glm::vec3 ro, glm::vec3 rd, glm::vec3 center, glm::vec3 WideH, float R, float* tout, float tbox )
{
	glm::vec3 innerWide = WideH - glm::vec3( R, R, R );
	glm::vec3 hitLocal = ( ro + rd * tbox - center );
	glm::vec3 distanceFromInner = glm::abs( hitLocal ) - innerWide;

	// Plane part, inclusive
	if( minElement( glm::max( distanceFromInner, { distanceFromInner.y, distanceFromInner.z, distanceFromInner.x } ) ) <= 0.0f )
	{
		if( 0.0f < tbox )
		{
			*tout = tbox;
			return true;
		}
		else
		{
			return false;
		}
	}

	glm::vec3 oLocal =
		{
			copysignf( innerWide.x, hitLocal.x ),
			copysignf( innerWide.y, hitLocal.y ),
			copysignf( innerWide.z, hitLocal.z ),
		};

	glm::vec3 rpo = ro - ( oLocal + center );

	float R2 = R * R;
	glm::vec3 rpo2 = rpo * rpo;
	glm::vec3 rpoxrd2 = rpo * rd;
	glm::vec3 rd2 = rd * rd;

	float t = FLT_MAX;

	// Brute force approach 4 spheres:
	//{ // corner sphere 1
	//	float a = rd2.x + rd2.y + rd2.z;
	//	float b = rpoxrd2.x + rpoxrd2.y + rpoxrd2.z;
	//	float c = rpo2.x + rpo2.y + rpo2.z - R2;
	//	float D = b * b - a * c;
	//	if( 0.0f < D )
	//	{
	//		float h = ( -b - sqrtf( D ) ) / a;
	//		glm::vec3 p = ro + rd * h - center;
	//		if( 0.0f < h && 0.0f <= minElement( glm::abs( p ) - innerWide ) /* leave only the corners, inclusive */ )
	//		{
	//			t = h;
	//		}
	//	}
	//}
	//for( glm::vec3 flipper : { glm::vec3{ -1.0f, 1.0f, 1.0f }, glm::vec3{ 1.0f, -1.0f, 1.0f }, glm::vec3{ 1.0f, 1.0f, -1.0f } } )
	//{ // corner sphere 2 ( 3 other potential spheres )
	//	glm::vec3 rpo = ro - ( oLocal * flipper + center );
	//	glm::vec3 rpo2 = rpo * rpo;
	//	glm::vec3 rpoxrd2 = rpo * rd;

	//	float a = rd2.x + rd2.y + rd2.z;
	//	float b = rpoxrd2.x + rpoxrd2.y + rpoxrd2.z;
	//	float c = rpo2.x + rpo2.y + rpo2.z - R2;
	//	float D = b * b - a * c;
	//	if( 0.0f < D )
	//	{
	//		float h = ( -b - sqrtf( D ) ) / a;
	//		glm::vec3 p = ro + rd * h - center;
	//		if( 0.0f < h && h < t && 0.0f <= minElement( glm::abs( p ) - innerWide ) /* leave only the corners, inclusive */ )
	//		{
	//			t = h;
	//		}
	//	}
	//}

	// if you find an intersection on the other side of oLocal on the cylinders, it means sphere is on the side. So that you must flip it. 
	glm::vec3 flipper = { 1, 1, 1 };

	{ // XY
		float a = rd2.x + rd2.y;
		float b = rpoxrd2.x + rpoxrd2.y;
		float c = rpo2.x + rpo2.y - R2;
		float D = b * b - a * c;
		if( 0.0f < D )
		{
			float h = ( -b - sqrtf( D ) ) / a;
			float z = ro.z + rd.z * h;
			bool isCorner =
				innerWide.x < glm::abs( ro.x + rd.x * h - center.x ) &&
				innerWide.y < glm::abs( ro.y + rd.y * h - center.y );
			if( 0.0f < h && h < t && glm::abs( z - center.z ) < innerWide.z && isCorner /* leave only the corners */ )
			{
				t = h;
			}

			flipper.z = glm::sign( z - center.z ) * glm::sign( oLocal.z );
		}
	}
	{ // YZ
		float a = rd2.y + rd2.z;
		float b = rpoxrd2.y + rpoxrd2.z;
		float c = rpo2.y + rpo2.z - R2;
		float D = b * b - a * c;
		if( 0.0f < D )
		{
			float h = ( -b - sqrtf( D ) ) / a;
			float x = ro.x + rd.x * h;
			bool isCorner =
				innerWide.y < glm::abs( ro.y + rd.y * h - center.y ) &&
				innerWide.z < glm::abs( ro.z + rd.z * h - center.z );
			if( 0.0f < h && h < t && glm::abs( x - center.x ) < innerWide.x && isCorner /* leave only the corners */ )
			{
				t = h;
			}

			flipper.x = glm::sign( x - center.x ) * glm::sign( oLocal.x );
		}
	}
	{ // ZX
		float a = rd2.z + rd2.x;
		float b = rpoxrd2.z + rpoxrd2.x;
		float c = rpo2.z + rpo2.x - R2;
		float D = b * b - a * c;
		if( 0.0f < D )
		{
			float h = ( -b - sqrtf( D ) ) / a;
			float y = ro.y + rd.y * h;
			bool isCorner =
				innerWide.x < glm::abs( ro.x + rd.x * h - center.x ) &&
				innerWide.z < glm::abs( ro.z + rd.z * h - center.z );
			if( 0.0f < h && h < t && glm::abs( y - center.y ) < innerWide.y && isCorner /* leave only the corners */ )
			{
				t = h;
			}
			flipper.y = glm::sign( y - center.y ) * glm::sign( oLocal.y );
		}
	}

	{ // corner sphere 
		glm::vec3 rpo = ro - ( oLocal * flipper + center );

		glm::vec3 rpo2 = rpo * rpo;
		glm::vec3 rpoxrd2 = rpo * rd;

		float a = rd2.x + rd2.y + rd2.z;
		float b = rpoxrd2.x + rpoxrd2.y + rpoxrd2.z;
		float c = rpo2.x + rpo2.y + rpo2.z - R2;
		float D = b * b - a * c;
		if( 0.0f < D )
		{
			float h = ( -b - sqrtf( D ) ) / a;
			glm::vec3 p = ro + rd * h - center;
			if( 0.0f < h && h < t && 0.0f <= minElement( glm::abs( p ) - innerWide ) /* leave only the corners, inclusive */ )
			{
				t = h;
			}
		}
	}

	if( t != FLT_MAX )
	{
		*tout = t;
		return true;
	}
	return false;
};
int main()
{
	using namespace pr;

	Config config;
	config.ScreenWidth = 1920;
	config.ScreenHeight = 1080;
	config.SwapInterval = 0;
	Initialize( config );

	Camera3D camera;
	camera.origin = { -4, -4, -4 };
	camera.lookat = { 0, 0, 0 };
	camera.zUp = false;
	camera.zNear = 0.5f;

	SetDataDir( ExecutableDir() );

	bool renderParallel = true;

	SetDepthTest( true );
	pr::ITexture* bgTexture = 0;

	while( pr::NextFrame() == false )
	{
		if( IsImGuiUsingMouse() == false )
		{
			UpdateCameraBlenderLike( &camera );
		}
		if( bgTexture )
		{
			ClearBackground( bgTexture );
		}
		else
		{
			ClearBackground( 0.1f, 0.1f, 0.1f, 1 );
		}

		BeginCamera( camera );

		PushGraphicState();

		DrawGrid( GridAxis::XZ, 1.0f, 10, { 128, 128, 128 } );
		DrawXYZAxis( 1.0f );

		Image2DRGBA8 image;
		image.allocate( GetScreenWidth(), GetScreenHeight() );

		CameraRayGenerator rayGenerator( GetCurrentViewMatrix(), GetCurrentProjMatrix(), image.width(), image.height() );
		static float Radius = 0.3f;

		static float Z = 0.0f;
		static float Y = -1.5f;
		static float X = 0;
		static float w = 1;

		static int debugX = 1000;
		static int debugY = 500;

		static glm::vec3 lower = { -0.5f, -0.5f, -0.5f };
		static glm::vec3 upper = { 0.5f, 0.5f, 0.5f };
		ManipulatePosition( camera, &lower, 0.5f );
		ManipulatePosition( camera, &upper, 0.5f );

		DrawAABB( lower, upper, { 64, 64, 64 } );
		Xoshiro128StarStar rng;
		for( int i = 0; i < 1000; i++ )
		{
			glm::vec3 ro = {
				X + glm::mix( -4.0f, 1.0f, rng.uniformf() * w ),
				Y,
				Z };
			glm::vec3 rd = { 0.08f, 1, 0.08f };
			float t = FLT_MAX;

			glm::vec3 one_over_rd = glm::vec3{ 1.0f, 1.0f, 1.0f } / rd;
			one_over_rd = glm::min( one_over_rd, glm::vec3( FLT_MAX ) );

			float tbox;
			if( hasIntersectionBox( ro, one_over_rd, lower, upper, &tbox ) )
			{
				if( intersectRoundedbox( ro, rd, ( lower + upper ) * 0.5f, ( upper - lower ) * 0.5f, Radius, &t, tbox ) )
				{
				}

				if( t != FLT_MAX )
				{
					DrawLine( ro, ro + rd * t, { 255, 0, 0 } );
				}
				else
				{
					DrawLine( ro, ro + rd * 10.f, { 64, 64, 64 } );
				}
			}
			else
			{
				DrawLine( ro, ro + rd * 10.f, { 64, 64, 64 } );
			}
		}

		auto renderLine = [&]( int j )
		{
			for( int i = 0; i < image.width(); ++i )
			{
				glm::vec3 ro, rd;
				rayGenerator.shoot( &ro, &rd, i, j, 0.5f, 0.5f );

				float t = FLT_MAX;

				glm::vec3 one_over_rd = glm::vec3{ 1.0f, 1.0f, 1.0f } / rd;
				one_over_rd = glm::min( one_over_rd, glm::vec3( FLT_MAX ) );

				glm::vec3 hitN;

				float tbox;
				if( hasIntersectionBox( ro, one_over_rd, lower, upper, &tbox ) )
				{
					glm::vec3 center = ( lower + upper ) * 0.5f;
					glm::vec3 sizeH = ( upper - lower ) * 0.5f;
					if( intersectRoundedbox( ro, rd, center, sizeH, Radius, &t, tbox ) )
					{
						// WARNING: It's not valid when R == 0
						glm::vec3 p = ro + rd * t;
						glm::vec3 dir = p - center;
						glm::vec3 nabs = glm::max( glm::abs( dir ) - ( sizeH - glm::vec3( Radius, Radius, Radius ) ), glm::vec3( 0, 0, 0 ) );
						hitN = glm::sign( dir ) * glm::normalize( nabs );
					}
				}
				if( t != FLT_MAX )
				{
					image( i, j ) = { 255, 255, 255, 255 };
					glm::vec3 color = ( hitN + glm::vec3{ 1.0f, 1.0f, 1.0f } ) * 0.5f;
					image( i, j ) = { 255 * color.x + 0.5f, 255 * color.y + 0.5f, 255 * color.z + 0.5f, 255 };
				}
				else
				{
					image( i, j ) = { 0, 0, 0, 255 };
				}

				if( debugX == i && debugY == j )
					image( i, j ) = { 255, 0, 0, 255 };
			}
		};
		//if( renderParallel )
		//{
		//	ParallelFor( image.height(), renderLine );
		//}
		//else
		{
			for( int j = 0; j < image.height(); ++j )
			{
				renderLine( j );
			}
		}
		if( bgTexture == nullptr )
		{
			bgTexture = CreateTexture();
		}
		bgTexture->upload( image );

		PopGraphicState();
		EndCamera();

		BeginImGui();

		ImGui::SetNextWindowSize( { 500, 800 }, ImGuiCond_Once );
		ImGui::Begin( "Panel" );
		ImGui::Text( "fps = %f", GetFrameRate() );
		ImGui::SliderFloat( "R", &Radius, 0, 1 );
		ImGui::SliderFloat( "Z", &Z, -3, 3 );
		ImGui::SliderFloat( "Y", &Y, -3, 3 );
		ImGui::SliderFloat( "X", &X, -3, 3 );
		ImGui::SliderFloat( "w", &w, 0, 1 );
		
		ImGui::End();

		EndImGui();
	}

	pr::CleanUp();
}