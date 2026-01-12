"""
Test script to validate proper error responses in the RAG Agent API
"""
import asyncio
import aiohttp
import json
from datetime import datetime

async def test_error_scenarios():
    """Test various error scenarios in the API"""
    base_url = "http://localhost:8000"

    print("Testing error scenarios for RAG Agent API...")

    # Test 1: Empty query
    print("\n1. Testing empty query...")
    try:
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{base_url}/api/chat",
                json={"query": "", "mode": "general", "filters": {}, "top_k": 5},
                headers={"Content-Type": "application/json"}
            )
            result = await response.json()
            print(f"   Status: {response.status}")
            print(f"   Response: {result}")
    except Exception as e:
        print(f"   Error: {str(e)}")

    # Test 2: Very short query (less than 3 characters)
    print("\n2. Testing very short query...")
    try:
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{base_url}/api/chat",
                json={"query": "hi", "mode": "general", "filters": {}, "top_k": 5},
                headers={"Content-Type": "application/json"}
            )
            result = await response.json()
            print(f"   Status: {response.status}")
            print(f"   Response: {result}")
    except Exception as e:
        print(f"   Error: {str(e)}")

    # Test 3: Invalid JSON format
    print("\n3. Testing invalid JSON format...")
    try:
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{base_url}/api/chat",
                data='{"invalid": json}',
                headers={"Content-Type": "application/json"}
            )
            result = await response.text()
            print(f"   Status: {response.status}")
            print(f"   Response: {result}")
    except Exception as e:
        print(f"   Error: {str(e)}")

    # Test 4: Rate limiting
    print("\n4. Testing rate limiting...")
    try:
        async with aiohttp.ClientSession() as session:
            # Make multiple rapid requests to trigger rate limiting
            tasks = []
            for i in range(15):  # More than the 10/minute limit
                task = session.post(
                    f"{base_url}/api/chat",
                    json={"query": f"test query {i}", "mode": "general", "filters": {}, "top_k": 5},
                    headers={"Content-Type": "application/json"}
                )
                tasks.append(task)

            responses = await asyncio.gather(*[await t for t in tasks], return_exceptions=True)

            rate_limited_count = 0
            for i, resp in enumerate(responses):
                if isinstance(resp, Exception):
                    print(f"   Request {i}: Failed with exception: {resp}")
                elif hasattr(resp, 'status') and resp.status == 429:
                    rate_limited_count += 1
                    print(f"   Request {i}: Rate limited (Status: {resp.status})")
                elif hasattr(resp, 'status'):
                    print(f"   Request {i}: Status {resp.status}")

            print(f"   Total rate limited requests: {rate_limited_count}")
    except Exception as e:
        print(f"   Error during rate limiting test: {str(e)}")

    # Test 5: Health check error scenario (if we simulate an error)
    print("\n5. Testing health check...")
    try:
        async with aiohttp.ClientSession() as session:
            response = await session.get(f"{base_url}/api/health")
            result = await response.json()
            print(f"   Status: {response.status}")
            print(f"   Response: {json.dumps(result, indent=2)}")
    except Exception as e:
        print(f"   Error: {str(e)}")

    print("\nError scenario testing completed.")


async def test_valid_requests():
    """Test that valid requests still work properly"""
    base_url = "http://localhost:8000"

    print("\nTesting valid requests...")

    try:
        async with aiohttp.ClientSession() as session:
            response = await session.post(
                f"{base_url}/api/chat",
                json={"query": "What is this book about?", "mode": "general", "filters": {}, "top_k": 5},
                headers={"Content-Type": "application/json"}
            )
            result = await response.json()
            print(f"   Valid request - Status: {response.status}")
            if response.status == 200:
                print(f"   Success: Got response with query_id: {result.get('query_id', 'N/A')}")
            else:
                print(f"   Error response: {result}")
    except Exception as e:
        print(f"   Error with valid request: {str(e)}")


if __name__ == "__main__":
    print("Starting API error scenario tests...")
    print("="*50)

    # Run tests
    asyncio.run(test_error_scenarios())
    asyncio.run(test_valid_requests())

    print("="*50)
    print("All tests completed.")