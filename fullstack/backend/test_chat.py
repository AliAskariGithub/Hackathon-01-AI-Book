"""Test script for chat endpoint with Groq integration."""
import requests
import json
import sys

def test_chat_endpoint():
    """Test the chat endpoint with a sample query."""

    url = "http://localhost:8000/api/chat"

    payload = {
        "query": "What is ROS 2?",
        "conversation_history": []
    }

    print("=" * 60)
    print("Testing Chat Endpoint with Groq Integration")
    print("=" * 60)
    print(f"\nEndpoint: {url}")
    print(f"Query: {payload['query']}")
    print("\nSending request...\n")

    try:
        response = requests.post(
            url,
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=30
        )

        print(f"Status Code: {response.status_code}")
        print("-" * 60)

        if response.status_code == 200:
            data = response.json()

            if data.get("error"):
                print(f"[ERROR] Error: {data['error']}")
                print(f"Conversation ID: {data.get('conversation_id')}")
                return False

            print("[SUCCESS] Response received!")
            print(f"\nConversation ID: {data.get('conversation_id')}")
            answer = data.get('answer', '')
            print(f"\nAnswer:\n{answer.encode('ascii', 'ignore').decode()}")

            citations = data.get('citations', [])
            if citations:
                print(f"\n[CITATIONS] ({len(citations)}):")
                for i, citation in enumerate(citations, 1):
                    title = citation.get('title', '').encode('ascii', 'ignore').decode()
                    print(f"  {i}. {title}")
                    print(f"     URL: {citation['url']}")
                    score = citation.get('score')
                    if score is not None:
                        print(f"     Score: {score:.3f}")
            else:
                print("\n[WARN] No citations found")

            return True
        else:
            print(f"[ERROR] HTTP Error: {response.status_code}")
            print(f"Response: {response.text}")
            return False

    except requests.exceptions.Timeout:
        print("[ERROR] Request timed out after 30 seconds")
        return False
    except requests.exceptions.ConnectionError as e:
        print(f"[ERROR] Connection error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        return False
    finally:
        print("\n" + "=" * 60)

if __name__ == "__main__":
    success = test_chat_endpoint()
    sys.exit(0 if success else 1)
